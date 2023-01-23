#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/base/TRef.h>
#include "artec_scanner_util.h"

namespace artec_scanner_robotraconteur_driver
{
    class ScanningProcedure;
    
    class ArtecScannerImpl : public experimental::artec_scanner::ArtecScanner_default_impl, 
        public RR_ENABLE_SHARED_FROM_THIS<ArtecScannerImpl>
    {

        private:
            artec::sdk::base::TRef<artec::sdk::capturing::IScanner> scanner = nullptr;
            artec::sdk::base::TRef<artec::sdk::capturing::IFrameProcessor> processor = nullptr;

            uint32_t add_workset(RRAlgorithmWorksetPtr workset);

            uint32_t handle_cnt = 100;
            std::map<uint32_t,RRAlgorithmWorksetPtr> worksets;

            boost::mutex this_lock;
            

        public:
            friend class ScanningProcedure;

            void Init(artec::sdk::capturing::IScanner* scanner);

            RR_INTRUSIVE_PTR<com::robotraconteur::geometry::shapes::Mesh > capture(RobotRaconteur::rr_bool with_texture) override;

            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::ScanningProcedureStatusPtr,void>
                run_scanning_procedure(const experimental::artec_scanner::ScanningProcedureSettingsPtr& settings) 
                override;

            virtual ~ArtecScannerImpl();
    };

    using ArtecScannerImplPtr = boost::shared_ptr<ArtecScannerImpl>;
    using ArtecScannerImplWeakPtr = boost::weak_ptr<ArtecScannerImpl>;
}
