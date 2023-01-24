#include "artec_scanning_procedure.h"
#include "artec_scanner_impl.h"
#include "artec_scanner_util.h"

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/io/ObjIO.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>
#include <artec/sdk/base/io/PngIO.h>
#include <artec/sdk/scanning/IScanningProcedure.h>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
    using namespace artec::sdk::scanning;
    // using namespace artec::sdk::algorithms;
};
using asdk::TRef;
using asdk::TArrayRef;

namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_image = com::robotraconteur::image;
namespace RR=RobotRaconteur;
namespace rr_artec = experimental::artec_scanner;
namespace rr_action = com::robotraconteur::action;

namespace artec_scanner_robotraconteur_driver
{

    boost::shared_ptr<ArtecScannerImpl> ScanningProcedure::GetParent()
    {
        auto p = parent.lock();
        if (!p) {
            RR_ARTEC_LOG_ERROR("ArtecScannerImpl parent has been released");
            throw RR::InvalidOperationException("ArtecScannerImpl parent has been released");
        }
        return p;
    }
    
    ScanningProcedure::ScanningProcedure(boost::shared_ptr<ArtecScannerImpl> parent)
    {
        this->parent = parent;
    }

    void ScanningProcedure::Init(const experimental::artec_scanner::ScanningProcedureSettingsPtr& settings)
    {
        RR_NULL_CHECK(settings);
        asdk::ScanningProcedureSettings desc = { 0 };
        desc.maxFrameCount = settings->max_frame_count;
        desc.registrationType = (asdk::RegistrationAlgorithmType)settings->registration_type;
        desc.pipelineConfiguration = settings->pipeline_configuration;
        desc.initialState = (asdk::ScanningState)settings->initial_state;
        desc.scanningCallback = new ScanningProcedureObserver(shared_from_this());
        desc.ignoreRegistrationErrors = settings->ignore_registration_errors.value != 0;
        desc.captureTexture = (asdk::CaptureTextureMethod)settings->capture_texture;
        desc.captureTextureFrequency = settings->capture_texture_frequency;
        desc.saveEmptySurfaces = settings->save_empty_surfaces.value != 0;

        RR_CALL_ARTEC(asdk::createScanningProcedure(&this->scanning_procedure, GetParent()->scanner, &desc), 
            "Error creating scanning procedure");

        model = boost::make_shared<RRArtecModel>();        
        RR_CALL_ARTEC(asdk::createModel(&input_container), "Error creating input model");
        RR_CALL_ARTEC(asdk::createCancellationTokenSource(&ct_source), "Error creating cancellation source");

        workset.in = input_container;
        workset.out = model->model;
        workset.cancellation = ct_source->getToken();
        workset.progress = nullptr;
        workset.threadsCount = 0;        
    }

    void ScanningProcedure::AsyncNext(boost::function<void(const experimental::artec_scanner::ScanningProcedureStatusPtr&,
        const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler, int32_t timeout )
    {
        boost::mutex::scoped_lock lock(this_lock);
        if (aborted)
        {
            throw RR::OperationAbortedException("Scanning Procedure operation was aborted");
        }
        if ((closed && !started) || completed)
        {
            throw RR::StopIterationException("");
        }

        if (!started)
        {
            auto job_observer = new ScanningProcedureJobObserver(shared_from_this());
            RR_CALL_ARTEC(asdk::launchJob(scanning_procedure, &workset, job_observer), 
                "Error launching scanning procedure");
            started = true;
            auto ret = rr_artec::ScanningProcedureStatusPtr(new rr_artec::ScanningProcedureStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->model_handle = 0;
            RR_ARTEC_LOG_INFO("Started scanning procedure")
            lock.unlock();
            handler(ret, nullptr);
            return;
        }

        if (next_handler)
        {
            throw RR::InvalidOperationException("Next call already in progress");
        }

        if (artec_job_complete)
        {
            complete_gen(handler);
            return;
        }

        next_handler = handler;

        RR_WEAK_PTR<ScanningProcedure> weak_this = shared_from_this();
        next_timer = RR::RobotRaconteurNode::s()->CreateTimer(boost::posix_time::seconds(5), 
            [weak_this](const RR::TimerEvent& evt) {
                auto t = weak_this.lock();
                if (!t) return;
                t->next_timer_handler(evt);
        }, true);
        next_timer->Start();
    }
        

    void ScanningProcedure::AsyncClose(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                    int32_t timeout)
    {
        boost::mutex::scoped_lock lock(this_lock);
        if (closed || aborted)
        {
            lock.unlock();
            handler(nullptr);
            return;
        }
        closed = true;
        if (started)
        {
            RR_ARTEC_LOG_INFO("Stopping scanner procedure from Close");
            RR_CALL_ARTEC(scanning_procedure->setState(asdk::ScanningState::ScanningState_Stop), 
                "Error stopping scanning procedure");
        }
        lock.unlock();
        handler(nullptr);
    }

    void ScanningProcedure::AsyncAbort(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                    int32_t timeout)
    {
        boost::mutex::scoped_lock lock(this_lock);
        if (closed || aborted)
        {
            lock.unlock();
            handler(nullptr);
            return;
        }
        aborted = true;
        if (started)
        {
            RR_ARTEC_LOG_INFO("Stopping scanner procedure from Abort");
            RR_CALL_ARTEC(scanning_procedure->setState(asdk::ScanningState::ScanningState_Stop), 
                "Error stopping scanner");
        }
        lock.unlock();
        handler(nullptr);
    }

    void ScanningProcedure::scan_job_complete(artec::sdk::base::ErrorCode result)
    {
        RR_ARTEC_LOG_INFO("Scanning procedure artec job complete: " << (int32_t)result);

        boost::mutex::scoped_lock lock(this_lock);
        artec_job_status = result;
        auto h = next_handler;
        next_handler.clear();
        if (h)
        {
            try
            {
                next_timer->Stop();
            }
            catch (std::exception&) {}

            complete_gen(h);
            return;
        }
    }

    void ScanningProcedure::complete_gen(boost::function<void(const experimental::artec_scanner::ScanningProcedureStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler)
    {
        RR_ARTEC_LOG_INFO("Completing scanning procedure generator");

        completed = true;

        if (artec_job_status != asdk::ErrorCode_OK)
        {

            auto exp = ArtecErrorToExceptionPtr(artec_job_status, "Scanning procedure failed");
            handler(nullptr, exp);
            return;
        }

        auto handle = GetParent()->add_model(model);
        auto ret = rr_artec::ScanningProcedureStatusPtr(new rr_artec::ScanningProcedureStatus());
        ret->action_status = rr_action::ActionStatusCode::complete;
        ret->model_handle = handle;
        handler(ret,nullptr);
    }

    void ScanningProcedure::next_timer_handler(const RR::TimerEvent& evt)
    {
        boost::mutex::scoped_lock lock(this_lock);
        auto h = next_handler;
        next_handler.clear();
        if (h)
        {
            auto ret = rr_artec::ScanningProcedureStatusPtr(new rr_artec::ScanningProcedureStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->model_handle = 0;
            h(ret, nullptr);
            return;
        }
    }


    void ScanningProcedureObserver::onFrameScanned(const artec::sdk::scanning::RegistrationInfo* frameInfo)
    {

    }
        
    void ScanningProcedureObserver::onFrameCaptured(const artec::sdk::scanning::RegistrationInfo* frameInfo)
    {

    }

    void ScanningProcedureObserver::onScanningFinished (int scannerIndex)
    {

    }

    ScanningProcedureObserver::ScanningProcedureObserver(boost::shared_ptr<ScanningProcedure> parent)
    {
        this->parent = parent;
    }

    ScanningProcedureJobObserver::ScanningProcedureJobObserver(boost::shared_ptr<ScanningProcedure> parent)
    {
        this->parent = parent;
    }

    void ScanningProcedureJobObserver::completed(artec::sdk::base::ErrorCode result)
    {
        boost::shared_ptr<ScanningProcedure> p = parent.lock();
        if (!p) return;
        p->scan_job_complete(result);
    }

}