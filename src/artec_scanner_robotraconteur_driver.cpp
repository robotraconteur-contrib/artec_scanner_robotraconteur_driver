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

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

namespace RR = RobotRaconteur;

using namespace artec_scanner_robotraconteur_driver;

int main(int argc, char* argv[])
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
    TRef<asdk::IScanner> scanner;
    ec = asdk::createScanner( &scanner, &defaultScanner );
    if( ec != asdk::ErrorCode_OK )
    {
        std::cerr << "Create scanner failed" << std::endl;
        return 2;
    }

    auto scanner_impl = RR_MAKE_SHARED<ArtecScannerImpl>();
    scanner_impl->Init(scanner);
    
    RR::ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, "experimental.artec_scanner", 64238);
    RR::RobotRaconteurNode::s()->RegisterService("scanner", "experimental.artec_scanner", scanner_impl);

    std::cout << "Press enter to quit..." << std::endl;
    getchar();

}
