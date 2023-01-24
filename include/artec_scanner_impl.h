#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/base/TRef.h>
#include "artec_scanner_util.h"

namespace artec_scanner_robotraconteur_driver
{
    class RRArtecModel : public experimental::artec_scanner::Model_default_impl
    {
    public:
        artec::sdk::base::TRef<artec::sdk::base::IModel> model;
        boost::mutex this_lock;
        bool busy = false;
        RRArtecModel();
    };

    using RRArtecModelPtr = boost::shared_ptr<RRArtecModel>;

    class ScanningProcedure;
    
    class ArtecScannerImpl : public experimental::artec_scanner::ArtecScanner_default_impl, 
        public RR_ENABLE_SHARED_FROM_THIS<ArtecScannerImpl>
    {

        private:
            artec::sdk::base::TRef<artec::sdk::capturing::IScanner> scanner = nullptr;
            artec::sdk::base::TRef<artec::sdk::capturing::IFrameProcessor> processor = nullptr;

            int32_t add_model(RRArtecModelPtr model);
                        
            int32_t handle_cnt = 100;
            std::map<int32_t,RRArtecModelPtr> models;

            boost::mutex this_lock;

            boost::optional<boost::filesystem::path> save_path;            

        public:
            friend class ScanningProcedure;

            void Init(artec::sdk::capturing::IScanner* scanner);

            void set_save_path(boost::optional<boost::filesystem::path> save_path);

            RR_INTRUSIVE_PTR<com::robotraconteur::geometry::shapes::Mesh > capture(RobotRaconteur::rr_bool with_texture) override;

            RobotRaconteur::RRArrayPtr<uint8_t> capture_obj(RobotRaconteur::rr_bool with_texture) override;

            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::ScanningProcedureStatusPtr,void>
                run_scanning_procedure(const experimental::artec_scanner::ScanningProcedureSettingsPtr& settings) 
                override;

            void model_free(int32_t model_handle) override;
            experimental::artec_scanner::ModelPtr get_models(int32_t model_handle) override;

            int32_t load_model(const std::string& project_name) override;

            void save_model(const std::string& project_name, int32_t model_handle) override;

            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::RunAlgorithmsStatusPtr,void >
                run_algorithms(const RobotRaconteur::RRListPtr<RobotRaconteur::RRValue>& algorithms, int32_t input_model_handle) override;

            virtual ~ArtecScannerImpl();
    };

    using ArtecScannerImplPtr = boost::shared_ptr<ArtecScannerImpl>;
    using ArtecScannerImplWeakPtr = boost::weak_ptr<ArtecScannerImpl>;

}
