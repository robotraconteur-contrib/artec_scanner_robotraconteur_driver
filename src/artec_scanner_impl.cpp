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
#include "artec_scanning_procedure.h"

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_image = com::robotraconteur::image;
namespace RR=RobotRaconteur;
namespace rr_artec = experimental::artec_scanner;

namespace artec_scanner_robotraconteur_driver
{

    RRArtecModel::RRArtecModel()
    {
        RR_CALL_ARTEC( asdk::createModel( &model ), "Error creating artec model");
    }

    void ArtecScannerImpl::Init(artec::sdk::capturing::IScanner* scanner)
    {
        this->scanner=scanner;
        RR_CALL_ARTEC(scanner->createFrameProcessor(&processor), "error creating frame processor");         

    }

    com::robotraconteur::geometry::shapes::MeshPtr ArtecScannerImpl::capture(RR::rr_bool with_texture)
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

    RR::RRArrayPtr<uint8_t> ArtecScannerImpl::capture_obj(RR::rr_bool with_texture)
    {
        throw RR::NotImplementedException("");
    }

    RR::GeneratorPtr<rr_artec::ScanningProcedureStatusPtr,void>
                ArtecScannerImpl::run_scanning_procedure(
                const rr_artec::ScanningProcedureSettingsPtr& settings)
    {
        auto proc = RR_MAKE_SHARED<ScanningProcedure>(shared_from_this());
        proc->Init(settings);
        RR_ARTEC_LOG_INFO("ScanningProcedure generator returned to client. Call Next() to begin.");
        return proc;
    }

    ArtecScannerImpl::~ArtecScannerImpl()
    {
        if (processor)
        {
            processor->release();
        }
    }

    int32_t ArtecScannerImpl::add_model(RRArtecModelPtr model)
    { 
        boost::mutex::scoped_lock lock(this_lock);
        auto h = ++handle_cnt;
        models.insert(std::make_pair(h,model));
        RR_ARTEC_LOG_INFO("Created model handle: " << h);
        return h;
    }

    rr_artec::ModelPtr ArtecScannerImpl::get_models(int32_t model_handle)
    {
        boost::mutex::scoped_lock lock(this_lock);

        auto e = models.find(model_handle);
        if (e == models.end())
        {
            RR_ARTEC_LOG_ERROR("Attempt to get invalid model: " << model_handle);
            throw RR::InvalidArgumentException("Invalid model handle");
        }
        return e->second;
    }

    /*RRAlgorithmWorksetPtr ArtecScannerImpl::get_workset_lock(uint32_t workset_handle, boost::mutex::scoped_try_lock& lock)
    {
        auto workset = get_workset(workset_handle);
        boost::mutex::scoped_try_lock workset_lock(workset->lock);
        if (!lock.owns_lock() || workset->busy)
        {
            RR_ARTEC_LOG_ERROR("Attempt to access workset while in use: " << workset_handle);
            throw RR::InvalidOperationException("Workset in use");
        }

        workset_lock.swap(lock);
        return workset;
    }*/

    void ArtecScannerImpl::model_free(int32_t model_handle)
    {
        boost::mutex::scoped_lock lock(this_lock);

        auto e = models.find(model_handle);
        if (e == models.end())
        {
            RR_ARTEC_LOG_ERROR("Attempt to free invalid model: " << model_handle);
            throw RR::InvalidArgumentException("Invalid workset handle");
        }

        models.erase(e);
    }

    

    RR::GeneratorPtr<rr_artec::RunAlgorithmsStatusPtr,void >
        ArtecScannerImpl::run_algorithms(const RR::RRListPtr<RR::RRValue>& algorithms, int32_t input_model_handle)
    {
        throw RR::NotImplementedException("Not implemented");
    }

}