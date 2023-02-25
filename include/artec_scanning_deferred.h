#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IScanningProcedureObserver.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IJobObserver.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include "artec_scanner_util.h" 

#include <boost/thread/thread_pool.hpp>
#include <list>

#pragma once

namespace artec_scanner_robotraconteur_driver
{
    class ArtecScannerImpl;
    struct RRDeferredCapture;

    class DeferredCapturePrepare : public RobotRaconteur::Generator<experimental::artec_scanner::DeferredCapturePrepareStatusPtr,void >,
        public RR_ENABLE_SHARED_FROM_THIS<DeferredCapturePrepare>
    {
        protected:
            boost::weak_ptr<ArtecScannerImpl> parent;
            boost::shared_ptr<ArtecScannerImpl> GetParent();
            boost::mutex& data_lock;
            boost::mutex this_lock;

            RobotRaconteur::TimerPtr next_timer;
            std::list<boost::shared_ptr<RRDeferredCapture> > input_data;

            boost::atomic<int32_t> completed_count = 0;
            boost::atomic<int32_t> failed_count = 0;

            bool mesh = false;
            bool stl = false;
            bool started = false;
            bool closed = false;
            bool aborted = false;
            bool completed = false;
            bool prepare_completed = false;
            size_t active_thread_count = 0;

            boost::function<void(const experimental::artec_scanner::DeferredCapturePrepareStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> next_handler;

            boost::thread_group thread_pool;
            artec::sdk::base::TRef<artec::sdk::capturing::IScanner> scanner;

        public:

            DeferredCapturePrepare(boost::shared_ptr<ArtecScannerImpl> parent);

            void Init(std::list<boost::shared_ptr<RRDeferredCapture> >&& input_data, bool mesh, bool stl);

            void AsyncNext(boost::function<void(const experimental::artec_scanner::DeferredCapturePrepareStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler, int32_t timeout = RR_TIMEOUT_INFINITE )
                override;

            void AsyncClose(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                            int32_t timeout = RR_TIMEOUT_INFINITE) override;

            void AsyncAbort(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                            int32_t timeout = RR_TIMEOUT_INFINITE) override;

            experimental::artec_scanner::DeferredCapturePrepareStatusPtr Next() override {return nullptr;}
            void Close() override {}
            void Abort() override {}

        protected:

            void complete_gen(boost::function<void(const experimental::artec_scanner::DeferredCapturePrepareStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler);

            void next_timer_handler(const RobotRaconteur::TimerEvent& evt);

            void prepare();
    };
}