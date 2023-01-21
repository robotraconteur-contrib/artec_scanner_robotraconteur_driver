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
        RR_CALL_ARTEC(scanner->createFrameProcessor(&processor), "error creating frame processor");         

    }

    RR_INTRUSIVE_PTR<com::robotraconteur::geometry::shapes::Mesh > ArtecScannerImpl::capture(RobotRaconteur::rr_bool with_texture)
    {
        RR_ARTEC_LOG_INFO("Begin scanner capture");
        TRef<asdk::IFrame> frame;
        TRef<asdk::IFrameMesh> mesh;
        frame = nullptr;
        mesh = nullptr;
        asdk::ErrorCode ec = asdk::ErrorCode_OK;
        RR_CALL_ARTEC(scanner->capture( &frame, with_texture.value != 0), "Error capturing from scanner");
        
        RR_CALL_ARTEC(processor->reconstructAndTexturizeMesh( &mesh, frame ), "Error reconstructing mesh");
        
        com::robotraconteur::geometry::shapes::MeshPtr rr_mesh = ConvertArtecMeshToRR(mesh);
        RR_ARTEC_LOG_INFO("Scanner capture complete");
        return rr_mesh;
    }

    ArtecScannerImpl::~ArtecScannerImpl()
    {
        if (processor)
        {
            processor->release();
        }
    }

}