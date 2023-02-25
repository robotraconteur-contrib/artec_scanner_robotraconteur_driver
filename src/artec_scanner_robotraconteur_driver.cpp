#include "robotraconteur_generated.h"
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
#include "artec_scanner_impl.h"

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/io/ObjIO.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>
#include <boost/program_options.hpp>
#include <artec/sdk/algorithms/Algorithms.h>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

namespace RR = RobotRaconteur;
namespace po = boost::program_options;

using namespace artec_scanner_robotraconteur_driver;

int main(int argc, char* argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("project-save-path", po::value<std::string>(), "set project save path")
        ("no-scanner","Do not search for scanner. Only used to process existing scan data");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    TRef<asdk::IScanner> scanner;
    if(vm.count("no-scanner") == 0)
    {
        asdk::setOutputLevel( asdk::VerboseLevel_Trace );
        asdk::ErrorCode ec = asdk::ErrorCode_OK;
        TRef<asdk::IArrayScannerId> scannersList;
        std::cerr << "Enumerating scanners... " << std::endl;
        ec = asdk::enumerateScanners( &scannersList );
        if( ec != asdk::ErrorCode_OK )
        {
            std::cerr << "Enumerating scanners failed" << std::endl;
            return 1;
        }
        int scanner_count = scannersList->getSize();
        if( scanner_count == 0 )
        {
            std::cerr << "No scanners found" << std::endl;
            return 3;
        }
        const asdk::ScannerId* idArray = scannersList->getPointer();
        const asdk::ScannerId& defaultScanner = idArray[0]; // just take the first available scanner
        std::wcerr 
            << L"Connecting to " << asdk::getScannerTypeName( defaultScanner.type ) 
            << L" scanner " << defaultScanner.serial << L"... "
        ;
        ec = asdk::createScanner( &scanner, &defaultScanner );
        if( ec != asdk::ErrorCode_OK )
        {
            std::cerr << "Create scanner failed" << std::endl;
            return 2;
        }
    }

    auto scanner_impl = RR_MAKE_SHARED<ArtecScannerImpl>();
    scanner_impl->Init(scanner);
    if (vm.count("project-save-path"))
    {
        boost::filesystem::path save_path(vm["project-save-path"].as<std::string>());
        scanner_impl->set_save_path(save_path);
    }
    
    RR::RobotRaconteurNodeSetup node_setup(RR::RobotRaconteurNode::sp(),
        ROBOTRACONTEUR_SERVICE_TYPES, "experimental.artec_scanner", 64238,
        RR::RobotRaconteurNodeSetupFlags_SERVER_DEFAULT | RR::RobotRaconteurNodeSetupFlags_JUMBO_MESSAGE, 
        RR::RobotRaconteurNodeSetupFlags_SERVER_DEFAULT_ALLOWED_OVERRIDE,
        argc, argv);
    RR::RobotRaconteurNode::s()->RegisterService("scanner", "experimental.artec_scanner", scanner_impl);
    if (!artec::sdk::algorithms::checkAlgorithmsPermission())
    {
        RR_ARTEC_LOG_WARNING("Artec Algorithms not available on this computer");
    }

    std::cout << "Press enter to quit..." << std::endl;
    getchar();

}
