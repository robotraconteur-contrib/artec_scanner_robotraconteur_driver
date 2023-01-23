#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IScanningProcedureObserver.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IJobObserver.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include "artec_scanner_util.h" 

#pragma once

namespace artec_scanner_robotraconteur_driver
{
    class ArtecScannerImpl;
    class ScanningProcedure;
    class ScanningProcedureObserver;
    class ScanningProcedureJobObserver;
    class ScanningProcedure : public RobotRaconteur::Generator<experimental::artec_scanner::ScanningProcedureStatusPtr,void>,
        public RR_ENABLE_SHARED_FROM_THIS<ScanningProcedure>
    {
        protected:
            boost::weak_ptr<ArtecScannerImpl> parent;
            boost::shared_ptr<ArtecScannerImpl> GetParent();
            boost::mutex this_lock;
            artec::sdk::base::TRef<artec::sdk::scanning::IScanningProcedure> scanning_procedure;
            bool started = false;
            bool closed = false;
            bool aborted = false;
            bool completed = false;
            bool artec_job_complete = false;
            artec::sdk::base::ErrorCode artec_job_status = artec::sdk::base::ErrorCode_UnknownExceptionType;
            boost::function<void(const experimental::artec_scanner::ScanningProcedureStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> next_handler;
            RRAlgorithmWorksetPtr workset;
            RobotRaconteur::TimerPtr next_timer;
        public:

            friend class ScanningProcedureObserver;
            friend class ScanningProcedureJobObserver;

            ScanningProcedure(boost::shared_ptr<ArtecScannerImpl> parent);

            void Init(const experimental::artec_scanner::ScanningProcedureSettingsPtr& settings);

            void AsyncNext(boost::function<void(const experimental::artec_scanner::ScanningProcedureStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler, int32_t timeout = RR_TIMEOUT_INFINITE )
                override;

            void AsyncClose(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                            int32_t timeout = RR_TIMEOUT_INFINITE) override;

            void AsyncAbort(boost::function<void(const RobotRaconteur::RobotRaconteurExceptionPtr& err)> handler,
                            int32_t timeout = RR_TIMEOUT_INFINITE) override;

            experimental::artec_scanner::ScanningProcedureStatusPtr Next() override {return nullptr;}
            void Close() override {}
            void Abort() override {}


        protected:
            void scan_job_complete(artec::sdk::base::ErrorCode result);

            void complete_gen(boost::function<void(const experimental::artec_scanner::ScanningProcedureStatusPtr&,
                const RobotRaconteur::RobotRaconteurExceptionPtr&)> handler);

            void next_timer_handler(const RobotRaconteur::TimerEvent& evt);
    };

    class ScanningProcedureObserver : public artec::sdk::scanning::ScanningProcedureObserverBase
    {
        boost::weak_ptr<ScanningProcedure> parent;

    public:
        ScanningProcedureObserver(boost::shared_ptr<ScanningProcedure> parent);

        void onFrameScanned(const artec::sdk::scanning::RegistrationInfo* frameInfo) override;
        
        void onFrameCaptured(const artec::sdk::scanning::RegistrationInfo* frameInfo) override;

        void onScanningFinished (int scannerIndex) override;
    };

    class ScanningProcedureJobObserver : public artec::sdk::base::JobObserverBase
    {
        boost::weak_ptr<ScanningProcedure> parent;

    public:
        ScanningProcedureJobObserver(boost::shared_ptr<ScanningProcedure> parent);

        void completed (artec::sdk::base::ErrorCode result) override;
    };

}