#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IJobObserver.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/algorithms/Algorithms.h>
#include "artec_scanner_util.h" 

#pragma once

namespace artec_scanner_robotraconteur_driver
{
    class ArtecScannerImpl;
    class RRArtecModel;
    class RunAlgorithms : public RobotRaconteur::Generator<experimental::artec_scanner::RunAlgorithmsStatusPtr,void>,
        public RR_ENABLE_SHARED_FROM_THIS<RunAlgorithms>
    {
        protected:
            boost::weak_ptr<ArtecScannerImpl> parent;
            boost::shared_ptr<ArtecScannerImpl> GetParent();
            boost::mutex this_lock;
            bool started = false;
            bool closed = false;
            bool aborted = false;
            bool completed = false;
            bool artec_job_complete = false;
            artec::sdk::base::ErrorCode artec_job_status = artec::sdk::base::ErrorCode_UnknownExceptionType;
            boost::shared_ptr<RRArtecModel> input_model;

            boost::shared_ptr<RRArtecModel> current_input_model;
            boost::shared_ptr<RRArtecModel> current_output_model;

            artec::sdk::base::AlgorithmWorkset current_workset;
            artec::sdk::base::TRef<artec::sdk::base::ICancellationTokenSource> ct_source;

            uint32_t current_algorithm = 0;
            uint32_t last_algorithm_update = 0;
            RobotRaconteur::TimerPtr next_timer;

            std::vector<artec::sdk::base::TRef<artec::sdk::algorithms::IAlgorithm> > artec_algorithms;

            boost::function<void(const experimental::artec_scanner::RunAlgorithmsStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> next_handler;

        public:
            friend class RunAlgorithmsJobObserver;

            RunAlgorithms(boost::shared_ptr<ArtecScannerImpl> parent);

            void Init(boost::shared_ptr<RRArtecModel> input_model, 
                const RobotRaconteur::RRListPtr<RobotRaconteur::RRValue>& algorithms);

            void AsyncNext(boost::function<void(const experimental::artec_scanner::RunAlgorithmsStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler, int32_t timeout = RR_TIMEOUT_INFINITE )
                override;

            void AsyncClose(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                            int32_t timeout = RR_TIMEOUT_INFINITE) override;

            void AsyncAbort(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                            int32_t timeout = RR_TIMEOUT_INFINITE) override;

            experimental::artec_scanner::RunAlgorithmsStatusPtr Next() override {return nullptr;}
            void Close() override {}
            void Abort() override {}

        protected:
            void algorithm_job_complete(artec::sdk::base::ErrorCode result, uint32_t job_number);

            void complete_gen(boost::function<void(const experimental::artec_scanner::RunAlgorithmsStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler);

            void next_timer_handler(const RobotRaconteur::TimerEvent& evt);
    };

    class RunAlgorithmsJobObserver : public artec::sdk::base::JobObserverBase
    {
        boost::shared_ptr<RunAlgorithms> parent;
        uint32_t job_number;

    public:
        RunAlgorithmsJobObserver(boost::shared_ptr<RunAlgorithms> parent, uint32_t job_number);

        void completed (artec::sdk::base::ErrorCode result) override;
    };

}
