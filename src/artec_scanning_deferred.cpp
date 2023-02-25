#include "artec_scanning_deferred.h"
#include "artec_scanner_impl.h"
#include "artec_scanner_util.h"

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IFrame.h>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;


namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_image = com::robotraconteur::image;
namespace rr_action = com::robotraconteur::action;
namespace RR=RobotRaconteur;
namespace rr_artec = experimental::artec_scanner;

namespace artec_scanner_robotraconteur_driver
{
    DeferredCapturePrepare::DeferredCapturePrepare(boost::shared_ptr<ArtecScannerImpl> parent)
        : data_lock(parent->this_lock)
    {
        this->scanner = parent->scanner;
        this->parent=parent;
    }

    void DeferredCapturePrepare::Init(std::list<boost::shared_ptr<RRDeferredCapture> >&& input_data, bool mesh, bool stl)
    {
        this->input_data = std::move(input_data);
        this->mesh = mesh;
        this->stl = stl;
    }

    boost::shared_ptr<ArtecScannerImpl> DeferredCapturePrepare::GetParent()
    {
        auto p = parent.lock();
        if (!p) {
            RR_ARTEC_LOG_ERROR("ArtecScannerImpl parent has been released");
            throw RR::InvalidOperationException("ArtecScannerImpl parent has been released");
        }
        return p;
    }

    
    void DeferredCapturePrepare::prepare()
    {
        auto this_ = shared_from_this();
        active_thread_count = boost::thread::hardware_concurrency();
        for( unsigned i = 0; i < boost::thread::hardware_concurrency(); i++ )
        {
            thread_pool.create_thread( [this_]
            {
                // Initialize a frame processor 
                TRef<asdk::IFrameProcessor> processor;
                if( this_->scanner->createFrameProcessor( &processor ) != asdk::ErrorCode_OK )
                {
                    return;
                }
                while(true)
                {
                    RRDeferredCapturePtr work;
                    {
                        boost::mutex::scoped_lock lock(this_->this_lock);
                        auto e = this_->input_data.begin();
                        if (e == this_->input_data.end() || this_->closed || this_->aborted)
                        {
                            this_->active_thread_count--;
                            if (this_->active_thread_count == 0)
                            {
                                this_->prepare_completed = true;
                                if (this_->next_handler)
                                {
                                    this_->complete_gen(this_->next_handler);
                                }
                            }
                            return;
                        }
                        work = *e;
                        this_->input_data.erase(e);
                    }

                    if ((!this_->mesh || work->mesh) && (!this_->stl || work->mesh_stl_bytes))
                    {
                        continue;
                    }

                    try
                    {
                        
                        TRef<asdk::IFrameProcessor> processor;
                        if ( this_->scanner->createFrameProcessor( &processor ) != asdk::ErrorCode_OK )
                        {
                            return;
                        }
                        asdk::TRef<asdk::IFrameMesh> frame_mesh;
                        RR_CALL_ARTEC(processor->reconstructAndTexturizeMesh(&frame_mesh, work->frame ), "Error reconstructing mesh");
                        
                        rr_shapes::MeshPtr rr_mesh;
                        if (this_->mesh)
                        {
                            rr_mesh = ConvertArtecFrameMeshToRR(frame_mesh);
                        }

                        RR::RRArrayPtr<uint8_t> stl_bytes;
                        if (this_->stl)
                        {
                            stl_bytes = ConvertArtecMeshToStlBytes(frame_mesh);
                        }

                        {
                            boost::mutex::scoped_lock work_lock(this_->data_lock);
                            if (this_->stl)
                            {
                                work->mesh_stl_bytes = stl_bytes;
                            }
                            if (this_->mesh)
                            {
                                work->mesh = rr_mesh;
                            }
                        }
                        this_->completed_count.fetch_add(1, boost::memory_order_relaxed);

                        RR_ARTEC_LOG_INFO("Completed preparing deferred capture handle " << work->handle);
                    }
                    catch (RR::RobotRaconteurException& exp)
                    {
                        RR_ARTEC_LOG_ERROR("Error preparing deferred frame handle " << work->handle << ": " << exp.what());
                        this_->failed_count.fetch_add(1, boost::memory_order_relaxed);
                    }
                    catch (std::exception& exp)
                    {
                        RR_ARTEC_LOG_ERROR("Error preparing deferred frame handle " << work->handle << ": " << exp.what());
                        this_->failed_count.fetch_add(1, boost::memory_order_relaxed);
                    }
                }
            });
        }
    }

    void DeferredCapturePrepare::AsyncNext(boost::function<void(const experimental::artec_scanner::DeferredCapturePrepareStatusPtr&,
        const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler, int32_t timeout)
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
            prepare();
            auto ret = rr_artec::DeferredCapturePrepareStatusPtr(new rr_artec::DeferredCapturePrepareStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->completed_count = completed_count;
            ret->failed_count = failed_count;
            RR_ARTEC_LOG_INFO("Started prepare deferred captures")
            lock.unlock();
            handler(ret, nullptr);
            return;
        }

        if (next_handler)
        {
            throw RR::InvalidOperationException("Next call already in progress");
        }

        if (prepare_completed)
        {
            complete_gen(handler);
            return;
        }

        next_handler = handler;

        RR_WEAK_PTR<DeferredCapturePrepare> weak_this = shared_from_this();
        next_timer = RR::RobotRaconteurNode::s()->CreateTimer(boost::posix_time::seconds(5), 
            [weak_this](const RR::TimerEvent& evt) {
                auto t = weak_this.lock();
                if (!t) return;
                t->next_timer_handler(evt);
        }, true);
        next_timer->Start();
    }

    void DeferredCapturePrepare::AsyncClose(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                    int32_t timeout)
    {
        boost::mutex::scoped_lock lock(this_lock);
        closed = true;
        lock.unlock();
        handler(nullptr);
    }

    void DeferredCapturePrepare::AsyncAbort(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                    int32_t timeout)
    {
        boost::mutex::scoped_lock lock(this_lock);
        aborted = true;
        lock.unlock();
        handler(nullptr);
    }

    void DeferredCapturePrepare::complete_gen(boost::function<void(const experimental::artec_scanner::DeferredCapturePrepareStatusPtr&,
        const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler)
    {
        RR_ARTEC_LOG_INFO("Completing prepare deferred captures");
        
        completed = true;

        auto ret = rr_artec::DeferredCapturePrepareStatusPtr(new rr_artec::DeferredCapturePrepareStatus());
        ret->action_status = rr_action::ActionStatusCode::complete;
        ret->completed_count = completed_count;
        ret->failed_count = failed_count;
        handler(ret,nullptr);
    }

    void DeferredCapturePrepare::next_timer_handler(const RobotRaconteur::TimerEvent& evt)
    {
        boost::mutex::scoped_lock lock(this_lock);
        auto h = next_handler;
        next_handler.clear();
        if (h)
        {
            auto ret = rr_artec::DeferredCapturePrepareStatusPtr(new rr_artec::DeferredCapturePrepareStatus());
            ret->action_status = rr_action::ActionStatusCode::running;
            ret->completed_count = completed_count;
            ret->failed_count = failed_count;
            h(ret, nullptr);
            return;
        }
    }

}
