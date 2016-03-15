/*
 * Copyright (c) 2014-2016 Kartik Kumar, Dinamica Srl (me@kartikkumar.com)
 * Copyright (c) 2014-2016 Abhishek Agrawal, Delft University of Technology (abhishek.agrawal@protonmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include <Atom/convertCartesianStateToTwoLineElements.hpp>

#include <SQLiteCpp/SQLiteCpp.h>

#include <Astro/astro.hpp>

#include <libsgp4/Tle.h>
#include <libsgp4/SGP4.h>
#include <libsgp4/DateTime.h>

#include "D2D/sgp4Fetch.hpp"
#include "D2D/lambertFetch.hpp"
#include "D2D/tools.hpp"

namespace d2d
{

//! Fetch details of specific debris-to-debris SGP4 transfer.
void fetchSGP4Transfer( const rapidjson::Document& config )
{
    // Verify config parameters. Exception is thrown if any of the parameters are missing.
    const sgp4FetchInput input = checkSGP4FetchInput( config );

    // Set gravitational parameter used by SGP4 targeter.
    const double earthGravitationalParameter = kMU;
    std::cout << "Earth gravitational parameter " << earthGravitationalParameter
              << " kg m^3 s_2" << std::endl;

    std::cout << std::endl;
    std::cout << "******************************************************************" << std::endl;
    std::cout << "                             Simulation                           " << std::endl;
    std::cout << "******************************************************************" << std::endl;
    std::cout << std::endl;

    // extract TLE's from the catalog (to be used for departure and arrival object's sgp4 propagated orbits and paths)
    std::cout << "Parsing TLE catalog ... " << std::endl;

    // Parse catalog and store TLE objects.
    std::ifstream catalogFile( input.catalogPath.c_str( ) );
    std::string catalogLine;

    // Check if catalog is 2-line or 3-line version.
    std::getline( catalogFile, catalogLine );
    const int tleLines = getTleCatalogType( catalogLine );

    // Reset file stream to start of file.
    catalogFile.seekg( 0, std::ios::beg );

    typedef std::vector< std::string > TleStrings;
    typedef std::vector< Tle > TleObjects;
    TleObjects tleObjects;

    while ( std::getline( catalogFile, catalogLine ) )
    {
        TleStrings tleStrings;
        removeNewline( catalogLine );
        tleStrings.push_back( catalogLine );
        std::getline( catalogFile, catalogLine );
        removeNewline( catalogLine );
        tleStrings.push_back( catalogLine );

        if ( tleLines == 3 )
        {
            std::getline( catalogFile, catalogLine );
            removeNewline( catalogLine );
            tleStrings.push_back( catalogLine );
            tleObjects.push_back( Tle( tleStrings[ 0 ], tleStrings[ 1 ], tleStrings[ 2 ] ) );
        }

        else if ( tleLines == 2 )
        {
            tleObjects.push_back( Tle( tleStrings[ 0 ], tleStrings[ 1 ] ) );
        }
    }

    catalogFile.close( );
    const double totalTleObjects = tleObjects.size( );
    std::cout << totalTleObjects << " TLE objects parsed from catalog!" << std::endl;

    std::cout << "Fetching transfer from database ... " << std::endl;

    // Connect to database and fetch metadata required to construct sgp4TransferInput object.
    SQLite::Database database( input.databasePath.c_str( ), SQLITE_OPEN_READONLY );

    // select transfer data from the sgp4_scanner_results table in database
    std::ostringstream sgp4TransferSelect;
    sgp4TransferSelect << "SELECT * FROM sgp4_scanner_results WHERE transfer_id = "
                   << input.transferId << ";";
    SQLite::Statement sgp4Query( database, sgp4TransferSelect.str( ) );

    // retrieve the data from the sgp4_scanner_results
    sgp4Query.executeStep( );

    const int sgp4TransferId                        = sgp4Query.getColumn( 0 );
    const int lambertTransferId                     = sgp4Query.getColumn( 1 );
    const double sgp4ArrivalPositionX               = sgp4Query.getColumn( 2 );
    const double sgp4ArrivalPositionY               = sgp4Query.getColumn( 3 );
    const double sgp4ArrivalPositionZ               = sgp4Query.getColumn( 4 );
    const double sgp4ArrivalVelocityX               = sgp4Query.getColumn( 5 );
    const double sgp4ArrivalVelocityY               = sgp4Query.getColumn( 6 );
    const double sgp4ArrivalVelocityZ               = sgp4Query.getColumn( 7 );
    const double sgp4ArrivalPositionErrorX          = sgp4Query.getColumn( 8 );
    const double sgp4ArrivalPositionErrorY          = sgp4Query.getColumn( 9 );
    const double sgp4ArrivalPositionErrorZ          = sgp4Query.getColumn( 10 );
    const double sgp4ArrivalPositionError           = sgp4Query.getColumn( 11 );
    const double sgp4ArrivalVelocityErrorX          = sgp4Query.getColumn( 12 );
    const double sgp4ArrivalVelocityErrorY          = sgp4Query.getColumn( 13 );
    const double sgp4ArrivalVelocityErrorZ          = sgp4Query.getColumn( 14 );
    const double sgp4ArrivalVelocityError           = sgp4Query.getColumn( 15 );

    std::cout << "SGP4 transfer successfully fetched from database!" << std::endl;

    // select transfer data from the lambert_scanner_results table in database
    std::ostringstream lambertTransferSelect;
    lambertTransferSelect << "SELECT * FROM lambert_scanner_results WHERE transfer_id = "
                   << lambertTransferId << ";";
    SQLite::Statement lambertQuery( database, lambertTransferSelect.str( ) );

    // retrieve the data from lambert_scanner_results
    lambertQuery.executeStep( );

    const int    departureObjectId                  = lambertQuery.getColumn( 1 );
    const int    arrivalObjectId                    = lambertQuery.getColumn( 2 );
    const double departureEpoch                     = lambertQuery.getColumn( 3 );
    const double timeOfFlight                       = lambertQuery.getColumn( 4 );
    const int    revolutions                        = lambertQuery.getColumn( 5 );
    const int    prograde                           = lambertQuery.getColumn( 6 );
    const double departurePositionX                 = lambertQuery.getColumn( 7 );
    const double departurePositionY                 = lambertQuery.getColumn( 8 );
    const double departurePositionZ                 = lambertQuery.getColumn( 9 );
    const double departureVelocityX                 = lambertQuery.getColumn( 10 );
    const double departureVelocityY                 = lambertQuery.getColumn( 11 );
    const double departureVelocityZ                 = lambertQuery.getColumn( 12 );
    const double arrivalPositionX                   = lambertQuery.getColumn( 19 );
    const double arrivalPositionY                   = lambertQuery.getColumn( 20 );
    const double arrivalPositionZ                   = lambertQuery.getColumn( 21 );
    const double arrivalVelocityX                   = lambertQuery.getColumn( 22 );
    const double arrivalVelocityY                   = lambertQuery.getColumn( 23 );
    const double arrivalVelocityZ                   = lambertQuery.getColumn( 24 );
    const double departureDeltaVX                   = lambertQuery.getColumn( 37 );
    const double departureDeltaVY                   = lambertQuery.getColumn( 38 );
    const double departureDeltaVZ                   = lambertQuery.getColumn( 39 );
    const double transferDeltaV                     = lambertQuery.getColumn( 43 );

    std::cout << "Lambert transfer successfully fetched from database!" << std::endl;

    std::cout << "Propagating transfer ... " << std::endl;

    // Compute and store transfer state history by propagating conic section (Kepler orbit).
    std::vector< double > transferDepartureState( 6 );
    transferDepartureState[ 0 ] = departurePositionX;
    transferDepartureState[ 1 ] = departurePositionY;
    transferDepartureState[ 2 ] = departurePositionZ;
    transferDepartureState[ 3 ] = departureVelocityX + departureDeltaVX;
    transferDepartureState[ 4 ] = departureVelocityY + departureDeltaVY;
    transferDepartureState[ 5 ] = departureVelocityZ + departureDeltaVZ;

    // compute and store transfer state history by sgp4 propagation
    DateTime transferDepartureEpoch( ( departureEpoch - 1721425.5 ) * TicksPerDay );
    Tle transferOrbitTle = atom::convertCartesianStateToTwoLineElements< double, std::vector< double > >( transferDepartureState,
                                                                                                          transferDepartureEpoch );
    std::cout << "transfer Orbit Tle computed successfully!" << std::endl;

    const StateHistory sgp4TransferPath = sampleSGP4Orbit( transferOrbitTle,
                                                           timeOfFlight,
                                                           input.outputSteps,
                                                           departureEpoch );

    std::cout << "Transfer propagated successfully!" << std::endl;

    std::cout << std::endl;
    std::cout << "******************************************************************" << std::endl;
    std::cout << "                               Output                             " << std::endl;
    std::cout << "******************************************************************" << std::endl;
    std::cout << std::endl;

    // Write metadata to file.
    std::ostringstream metadataPath;
    metadataPath << input.outputDirectory << "/transfer" << input.transferId << "_"
                 << input.metadataFilename;
    std::ofstream metadataFile( metadataPath.str( ).c_str( ) );
    print( metadataFile, "departure_id", departureObjectId, "-" );
    metadataFile << std::endl;
    print( metadataFile, "arrival_id", arrivalObjectId, "-" );
    metadataFile << std::endl;
    print( metadataFile, "departure_epoch", departureEpoch, "JD" );
    metadataFile << std::endl;
    print( metadataFile, "time_of_flight", timeOfFlight, "s" );
    metadataFile << std::endl;

    if ( prograde == 1 )
    {
        print( metadataFile, "is_prograde", "true", "-" );
    }
    else
    {
        print( metadataFile, "is_prograde", "false", "-" );
    }
    metadataFile << std::endl;

    print( metadataFile, "Lambert revolutions", revolutions, "-" );
    metadataFile << std::endl;
    print( metadataFile, "Lambert transfer_delta_v", transferDeltaV, "km/s" );
    metadataFile << std::endl;
    print( metadataFile, "Position error magnitude", sgp4ArrivalPositionError, "km" );
    metadataFile << std::endl;
    print( metadataFile, "Velocity error magnitude", sgp4ArrivalVelocityError, "km/s" );
    metadataFile.close( );

    // Defined common header line for all the ephemeris files generated below.
    const std::string ephemerisFileHeader = "jd,x,y,z,xdot,ydot,zdot";

    Vector6 departureState;
    departureState[ 0 ] = departurePositionX;
    departureState[ 1 ] = departurePositionY;
    departureState[ 2 ] = departurePositionZ;
    departureState[ 3 ] = departureVelocityX;
    departureState[ 4 ] = departureVelocityY;
    departureState[ 5 ] = departureVelocityZ;

    // Compute period of departure orbit.
    const Vector6 departureStateKepler
        = astro::convertCartesianToKeplerianElements( departureState,
                                                      earthGravitationalParameter );
    const double departureOrbitalPeriod
        = astro::computeKeplerOrbitalPeriod( departureStateKepler[ astro::semiMajorAxisIndex ],
                                             earthGravitationalParameter );

    // Get departure object's TLE from the parsed TLE catalog
    bool departureTleFound = false;
    int departureTleIndex = 0;
    Tle departureOrbitTle;
    while ( departureTleFound == false && departureTleIndex < totalTleObjects )
    {
        if ( tleObjects[ departureTleIndex ].NoradNumber( ) == departureObjectId )
        {
            departureTleFound = true;
            departureOrbitTle = tleObjects[ departureTleIndex ];
        }
        ++departureTleIndex;
    }    

    // Sample sgp4 departure orbit.
    const StateHistory sgp4DepartureOrbit = sampleSGP4Orbit( departureOrbitTle,
                                                             departureOrbitalPeriod,
                                                             input.outputSteps,
                                                             departureEpoch );

    // Write sampled sgp4 departure orbit to file.
    std::ostringstream sgp4DepartureOrbitFilePath;
    sgp4DepartureOrbitFilePath << input.outputDirectory << "/transfer" << input.transferId << "_"
                           << input.departureOrbitFilename;
    std::ofstream sgp4DepartureOrbitFile( sgp4DepartureOrbitFilePath.str( ).c_str( ) );
    print( sgp4DepartureOrbitFile, sgp4DepartureOrbit, ephemerisFileHeader );
    sgp4DepartureOrbitFile.close( );

    // Sample sgp4 departure path.
    const StateHistory sgp4DeparturePath = sampleSGP4Orbit( departureOrbitTle,
                                                            timeOfFlight,
                                                            input.outputSteps,
                                                            departureEpoch );
    // Write sampled departure path to file.
    std::ostringstream sgp4DeparturePathFilePath;
    sgp4DeparturePathFilePath << input.outputDirectory << "/transfer" << input.transferId << "_"
                          << input.departurePathFilename;
    std::ofstream sgp4DeparturePathFile( sgp4DeparturePathFilePath.str( ).c_str( ) );
    print( sgp4DeparturePathFile, sgp4DeparturePath, ephemerisFileHeader );
    sgp4DeparturePathFile.close( );

    Vector6 arrivalState;
    arrivalState[ 0 ] = arrivalPositionX;
    arrivalState[ 1 ] = arrivalPositionY;
    arrivalState[ 2 ] = arrivalPositionZ;
    arrivalState[ 3 ] = arrivalVelocityX;
    arrivalState[ 4 ] = arrivalVelocityY;
    arrivalState[ 5 ] = arrivalVelocityZ;

    // Compute period of arrival orbit.
    const Vector6 arrivalStateKepler
        = astro::convertCartesianToKeplerianElements( arrivalState,
                                                      earthGravitationalParameter );
    const double arrivalOrbitalPeriod
        = astro::computeKeplerOrbitalPeriod( arrivalStateKepler[ astro::semiMajorAxisIndex ],
                                             earthGravitationalParameter );

    // Get arrival object's TLE from the parsed TLE catalog
    bool arrivalTleFound = false;
    int arrivalTleIndex = 0;
    Tle arrivalOrbitTle;
    while ( arrivalTleFound == false && arrivalTleIndex < totalTleObjects )
    {
        if ( tleObjects[ arrivalTleIndex ].NoradNumber( ) == arrivalObjectId )
        {
            arrivalTleFound = true;
            arrivalOrbitTle = tleObjects[ arrivalTleIndex ]; 
        }
        ++arrivalTleIndex;
    }    

    // Sample sgp4 arrival orbit.
    const StateHistory sgp4ArrivalOrbit = sampleSGP4Orbit( arrivalOrbitTle,
                                                           timeOfFlight,
                                                           input.outputSteps,
                                                           departureEpoch );

    // Write sampled sgp4 arrival orbit to file.
    std::ostringstream sgp4ArrivalOrbitFilePath;
    sgp4ArrivalOrbitFilePath << input.outputDirectory << "/transfer" << input.transferId << "_"
                         << input.arrivalOrbitFilename;
    std::ofstream sgp4ArrivalOrbitFile( sgp4ArrivalOrbitFilePath.str( ).c_str( ) );
    print( sgp4ArrivalOrbitFile, sgp4ArrivalOrbit, ephemerisFileHeader );
    sgp4ArrivalOrbitFile.close( );

    // Sample arrival path.
    const StateHistory sgp4ArrivalPath = sampleSGP4Orbit( arrivalOrbitTle,
                                                          timeOfFlight,
                                                          input.outputSteps,
                                                          departureEpoch );

    // Write sampled arrival path to file.
    std::ostringstream sgp4ArrivalPathFilePath;
    sgp4ArrivalPathFilePath << input.outputDirectory << "/transfer" << input.transferId << "_"
                        << input.arrivalPathFilename;
    std::ofstream sgp4ArrivalPathFile( sgp4ArrivalPathFilePath.str( ).c_str( ) );
    print( sgp4ArrivalPathFile, sgp4ArrivalPath, ephemerisFileHeader );
    sgp4ArrivalPathFile.close( );

    // Sample transfer trajectory.

    // Compute period of transfer orbit.
    Vector6 transferDepartureStateArray;
    for (int i = 0; i < 6; i++ )
        transferDepartureStateArray[ i ] = transferDepartureState[ i ];

    const Vector6 transferDepartureStateKepler
        = astro::convertCartesianToKeplerianElements( transferDepartureStateArray,
                                                      earthGravitationalParameter );
    const double transferOrbitalPeriod
        = astro::computeKeplerOrbitalPeriod(
            transferDepartureStateKepler[ astro::semiMajorAxisIndex ],
            earthGravitationalParameter );

    // Sample transfer orbit.
    const StateHistory sgp4TransferOrbit = sampleSGP4Orbit( transferOrbitTle,
                                                            transferOrbitalPeriod,
                                                            input.outputSteps,
                                                            departureEpoch );


    // Write sampled transfer orbit to file.
    std::ostringstream sgp4TransferOrbitFilePath;
    sgp4TransferOrbitFilePath << input.outputDirectory << "/transfer" << input.transferId << "_"
                          << input.transferOrbitFilename;
    std::ofstream sgp4TransferOrbitFile( sgp4TransferOrbitFilePath.str( ).c_str( ) );
    print( sgp4TransferOrbitFile, sgp4TransferOrbit, ephemerisFileHeader );
    sgp4TransferOrbitFile.close( );

    // Write sampled transfer path to file.
    std::ostringstream sgp4TransferPathFilePath;
    sgp4TransferPathFilePath << input.outputDirectory << "/transfer" << input.transferId << "_"
                         << input.transferPathFilename;
    std::ofstream sgp4TransferPathFile( sgp4TransferPathFilePath.str( ).c_str( ) );
    print( sgp4TransferPathFile, sgp4TransferPath, ephemerisFileHeader );
    sgp4TransferPathFile.close( );
}

//! Check input parameters for sgp4_fetch.
sgp4FetchInput checkSGP4FetchInput( const rapidjson::Document& config )
{
    const std::string databasePath = find( config, "database" )->value.GetString( );
    std::cout << "Database                      " << databasePath << std::endl;

    const std::string catalogPath = find( config, "catalog" )->value.GetString( );
    std::cout << "Catalog                      " << catalogPath << std::endl;

    const int transferId = find( config, "transfer_id" )->value.GetInt( );
    std::cout << "Transfer ID                   " << transferId << std::endl;

    const int outputSteps = find( config, "output_steps" )->value.GetInt( );
    std::cout << "Output steps                  " << outputSteps << std::endl;

    const std::string outputDirectory = find( config, "output_directory" )->value.GetString( );
    std::cout << "Output directory              " << outputDirectory << std::endl;

    const std::string metadataFilename = find( config, "metadata" )->value.GetString( );
    std::cout << "Metadata file                 " << metadataFilename << std::endl;

    const std::string departureOrbitFilename
        = find( config, "departure_orbit" )->value.GetString( );
    std::cout << "Departure orbit file          " << departureOrbitFilename << std::endl;

    const std::string departurePathFilename
        = find( config, "departure_path" )->value.GetString( );
    std::cout << "Departure path file           " << departurePathFilename << std::endl;

    const std::string arrivalOrbitFilename = find( config, "arrival_orbit" )->value.GetString( );
    std::cout << "Arrival orbit file            " << arrivalOrbitFilename << std::endl;

    const std::string arrivalPathFilename = find( config, "arrival_path" )->value.GetString( );
    std::cout << "Arrival path file             " << arrivalPathFilename << std::endl;

    const std::string transferOrbitFilename = find( config, "transfer_orbit" )->value.GetString( );
    std::cout << "Transfer orbit file           " << transferOrbitFilename << std::endl;

    const std::string transferPathFilename = find( config, "transfer_path" )->value.GetString( );
    std::cout << "Transfer path file            " << transferPathFilename << std::endl;

    return sgp4FetchInput( databasePath,
                           catalogPath, 
                           transferId,
                           outputSteps,
                           outputDirectory,
                           metadataFilename,
                           departureOrbitFilename,
                           departurePathFilename,
                           arrivalOrbitFilename,
                           arrivalPathFilename,
                           transferOrbitFilename,
                           transferPathFilename );
}

} // namespace d2d
