#include "artec_scanner_algorithm.h"
#include "artec_scanner_algorithm_util.h"
#include "artec_scanner_impl.h"

#include <artec/sdk/algorithms/Algorithms.h>
#include <artec/sdk/base/IScan.h>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::algorithms;
};
using asdk::TRef;

namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_image = com::robotraconteur::image;
namespace RR=RobotRaconteur;
namespace rr_artec = experimental::artec_scanner;
namespace rr_action = com::robotraconteur::action;

#define RR_ARTEC_PREFIX "experimental.artec_scanner."

namespace artec_scanner_robotraconteur_driver
{

boost::shared_ptr<ArtecScannerImpl> RunAlgorithms::GetParent()
{
    auto p = parent.lock();
    if (!p) {
        RR_ARTEC_LOG_ERROR("ArtecScannerImpl parent has been released");
        throw RR::InvalidOperationException("ArtecScannerImpl parent has been released");
    }
    return p;
}

RunAlgorithms::RunAlgorithms(boost::shared_ptr<ArtecScannerImpl> parent)
{
    this->parent = parent;
}

void RunAlgorithms::Init(boost::shared_ptr<RRArtecModel> input_model, 
    const RobotRaconteur::RRListPtr<RobotRaconteur::RRValue>& algorithms)
{
    if (input_model->model->getSize() <= 0)
    {
        RR_ARTEC_LOG_ERROR("Model passed to run_algorithms does not contain any scans")
        throw RR::InvalidArgumentException("Model passed to run_algorithms does not contain any scans");
    }

    this->input_model = input_model;

    auto scanner_type = input_model->model->getElement(0)->getScannerType();

    std::vector<asdk::TRef<asdk::IAlgorithm> > artec_algs;
    for(auto& alg : *algorithms)
    {
        asdk::TRef<asdk::IAlgorithm> artec_alg;
        auto alg_rr_type = alg->RRType();
        // Look at type of each algorithm and dispatch appropriately
        if (alg_rr_type == (RR_ARTEC_PREFIX "AutoAlignAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::AutoAlignAlgorithm>(alg);
            create_auto_align_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "FastFusionAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::FastFusionAlgorithm>(alg);
            create_fast_fusion_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "FastMeshSimplificationAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::FastMeshSimplificationAlgorithm>(alg);
            create_fast_mesh_simplification_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "GlobalRegistrationAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::GlobalRegistrationAlgorithm>(alg);
            create_global_registration_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "LoopClosureAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::LoopClosureAlgorithm>(alg);
            create_loop_closure_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "MeshSimplificationAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::MeshSimplificationAlgorithm>(alg);
            create_mesh_simplification_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "OutliersRemovalAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::OutliersRemovalAlgorithm>(alg);
            create_outliers_removal_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "PoissonFusionAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::PoissonFusionAlgorithm>(alg);
            create_poisson_fusion_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "SerialRegistrationAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::SerialRegistrationAlgorithm>(alg);
            create_serial_registration_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "SmallObjectsFilterAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::SmallObjectsFilterAlgorithm>(alg);
            create_small_objects_filter_algorithm(&artec_alg, alg2, scanner_type); 
        }
        else
        if (alg_rr_type == (RR_ARTEC_PREFIX "TexturizationAlgorithm"))
        {
            auto alg2 = RR_DYNAMIC_POINTER_CAST<rr_artec::TexturizationAlgorithm>(alg);
            create_texturization_algorithm(&artec_alg, alg2, scanner_type); 
        }
        

        if (!artec_alg)
        {
            RR_ARTEC_LOG_ERROR("Invalid algorithm type: " << alg_rr_type);
            throw RR::InvalidArgumentException("Invalid algorithm type: " + alg_rr_type);
        }

        artec_algs.push_back(std::move(artec_alg));
    }

    if (artec_algs.empty())
    {
        RR_ARTEC_LOG_ERROR("No algorithms specified to run_algorithms");
        throw RR::InvalidArgumentException("No algorithms specified");
    }

    artec_algorithms.swap(artec_algs);
}

void RunAlgorithms::AsyncNext(boost::function<void(const experimental::artec_scanner::RunAlgorithmsStatusPtr&,
    const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler, int32_t timeout)
{
    boost::mutex::scoped_lock lock(this_lock);
        if (aborted)
        {
            throw RR::OperationAbortedException("Scanning Procedure operation was aborted");
        }
        if (closed || completed)
        {
            throw RR::StopIterationException("");
        }

        if (!started)
        {
            RR_CALL_ARTEC(asdk::createCancellationTokenSource(&ct_source), "Error creating cancellation source");
            auto job_observer = new RunAlgorithmsJobObserver(shared_from_this(), 0);
            auto job = artec_algorithms.at(0);
            current_input_model = input_model;
            current_output_model = RR_MAKE_SHARED<RRArtecModel>();
            current_workset.in = current_input_model->model;
            current_workset.out = current_output_model->model;
            current_workset.cancellation = ct_source->getToken();
            current_workset.progress = nullptr;
            current_workset.threadsCount = 0;

            RR_CALL_ARTEC(asdk::launchJob(job, &current_workset, job_observer), 
                "Error launching scanning procedure");
            started = true;
            auto ret = rr_artec::RunAlgorithmsStatusPtr(new rr_artec::RunAlgorithmsStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->output_model_handle = 0;
            ret->current_algorithm = 0;
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

        if (last_algorithm_update < current_algorithm)
        {
            auto ret = rr_artec::RunAlgorithmsStatusPtr(new rr_artec::RunAlgorithmsStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->output_model_handle = 0;
            ret->current_algorithm = current_algorithm;
            last_algorithm_update = current_algorithm;
            lock.unlock();
            handler(ret, nullptr);
            return;
        }

        next_handler = handler;

        RR_WEAK_PTR<RunAlgorithms> weak_this = shared_from_this();
        next_timer = RR::RobotRaconteurNode::s()->CreateTimer(boost::posix_time::seconds(5), 
            [weak_this](const RR::TimerEvent& evt) {
                auto t = weak_this.lock();
                if (!t) return;
                t->next_timer_handler(evt);
        }, true);
        next_timer->Start();
    }

void RunAlgorithms::AsyncClose(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
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
    this->ct_source->cancel();
    lock.unlock();
    handler(nullptr);
}

void RunAlgorithms::AsyncAbort(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
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
    this->ct_source->cancel();
    lock.unlock();
    handler(nullptr);
}

void RunAlgorithms::algorithm_job_complete(artec::sdk::base::ErrorCode result, uint32_t job_number)
{
    boost::mutex::scoped_lock lock(this_lock);
    auto next_algorithm = current_algorithm+1;
    
    if ((next_algorithm) < artec_algorithms.size() && result == asdk::ErrorCode_OK)
    {
        auto job = artec_algorithms.at(next_algorithm);
        // Feed output of last algorithm as input to next algorithm
        current_input_model = current_output_model;
        current_output_model = RR_MAKE_SHARED<RRArtecModel>();
        current_workset.in = current_input_model->model;
        current_workset.out = current_output_model->model;

        auto job_observer = new RunAlgorithmsJobObserver(shared_from_this(), next_algorithm);
        RR_CALL_ARTEC(asdk::launchJob(job, &current_workset, job_observer), 
                "Error launching next algorithm");
        
        current_algorithm = next_algorithm;
        auto h = next_handler;
        next_handler.clear();
        if (h)
        {
            auto ret = rr_artec::RunAlgorithmsStatusPtr(new rr_artec::RunAlgorithmsStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->output_model_handle = 0;
            ret->current_algorithm = current_algorithm;
            last_algorithm_update = current_algorithm;
            lock.unlock();
            h(ret, nullptr);
            return;
        }
    }
    else
    {
        artec_job_complete = true;
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
}

void RunAlgorithms::complete_gen(boost::function<void(const experimental::artec_scanner::RunAlgorithmsStatusPtr&,
    const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler)
{
    RR_ARTEC_LOG_INFO("Run Algorithms execution complete");

    completed = true;

    if (artec_job_status != asdk::ErrorCode_OK)
    {
        auto exp = ArtecErrorToExceptionPtr(artec_job_status, "Run Algorithms execution failed for algorithm: " + current_algorithm);
        handler(nullptr, exp);
        return;
    }

    auto handle = GetParent()->add_model(current_output_model);
    auto ret = rr_artec::RunAlgorithmsStatusPtr(new rr_artec::RunAlgorithmsStatus());
    ret->action_status = rr_action::ActionStatusCode::complete;
    ret->output_model_handle = handle;
    ret->current_algorithm = current_algorithm;
    handler(ret,nullptr);
}

void RunAlgorithms::next_timer_handler(const RobotRaconteur::TimerEvent& evt)
{
    boost::mutex::scoped_lock lock(this_lock);
    auto h = next_handler;
    next_handler.clear();
    if (h)
    {
        auto ret = rr_artec::RunAlgorithmsStatusPtr(new rr_artec::RunAlgorithmsStatus());
        ret->action_status = rr_action::ActionStatusCode::running;
        ret->output_model_handle = 0;
        ret->current_algorithm = current_algorithm;
        last_algorithm_update = current_algorithm;
        h(ret, nullptr);
        return;
    }
}

RunAlgorithmsJobObserver::RunAlgorithmsJobObserver(boost::shared_ptr<RunAlgorithms> parent, uint32_t job_number)
{
    this->parent = parent;
    this->job_number = job_number;
}

void RunAlgorithmsJobObserver::completed(artec::sdk::base::ErrorCode result)
{
    auto p = parent;
    parent.reset();
    if (!p) return;
    p->algorithm_job_complete(result, job_number);
}

}
