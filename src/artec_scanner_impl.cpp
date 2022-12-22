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

#include "artec_scanner_util.h"

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

namespace RR = RobotRaconteur;

namespace artec_scanner_robotraconteur_driver
{
    void ArtecScannerImpl::Init(artec::sdk::capturing::IScanner* scanner)
    {
        this->scanner=scanner;
         asdk::ErrorCode ec = asdk::ErrorCode_OK;
         ec = scanner->createFrameProcessor(&processor);
         if( ec != asdk::ErrorCode_OK )
         {
            throw RR::SystemResourceException("Could not create frame processor: " + boost::lexical_cast<std::string>(ec));
         }

    }

    RR_INTRUSIVE_PTR<com::robotraconteur::geometry::shapes::Mesh > ArtecScannerImpl::capture(RobotRaconteur::rr_bool with_texture)
    {
        TRef<asdk::IFrame> frame;
        TRef<asdk::IFrameMesh> mesh;
        frame = nullptr;
        mesh = nullptr;
        asdk::ErrorCode ec = asdk::ErrorCode_OK;
        ec = scanner->capture( &frame, with_texture.value != 0);
        if( ec != asdk::ErrorCode_OK )
        {
            throw RR::OperationFailedException("Capture frame failed: " + boost::lexical_cast<std::string>(ec));
        }

        ec = processor->reconstructAndTexturizeMesh( &mesh, frame );
        if( ec != asdk::ErrorCode_OK )
        {
            throw RR::OperationFailedException("Reconstruct and texturize mesh failed: " + boost::lexical_cast<std::string>(ec));
        }


        return ConvertArtecMeshToRR(mesh);
    }

    ArtecScannerImpl::~ArtecScannerImpl()
    {
        if (processor)
        {
            processor->release();
        }
    }

}