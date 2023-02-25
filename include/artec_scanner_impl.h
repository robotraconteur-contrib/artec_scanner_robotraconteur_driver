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

        RRArtecModel();

        uint32_t get_scan_count() override;

        experimental::artec_scanner::ScanPtr get_scans(int32_t ind) override;

        RobotRaconteur::rr_bool get_composite_container_valid() override;

        experimental::artec_scanner::CompositeContainerPtr get_composite_container() override;
    };
    using RRArtecModelPtr = boost::shared_ptr<RRArtecModel>;

    class RRScan : public experimental::artec_scanner::Scan_default_impl
    {  
    public:
        artec::sdk::base::IScan* scan;

        RRScan(artec::sdk::base::IScan* scan);

        com::robotraconteur::geometry::Transform get_scan_transform() override;
        uint32_t get_frame_count() override;
        com::robotraconteur::geometry::shapes::MeshPtr getf_frame_mesh(uint32_t ind) override;

        RobotRaconteur::RRArrayPtr<uint8_t > getf_frame_mesh_stl(uint32_t ind) override;

        com::robotraconteur::geometry::Transform getf_frame_transform(uint32_t ind) override;
    };

    class RRCompositeContainer : public experimental::artec_scanner::CompositeContainer
    {
    public:
        artec::sdk::base::ICompositeContainer *container;

        RRCompositeContainer(artec::sdk::base::ICompositeContainer *container);

        uint32_t get_composite_mesh_count() override;
        com::robotraconteur::geometry::Transform get_composite_container_transform() override;
        com::robotraconteur::geometry::shapes::MeshPtr getf_composite_mesh(uint32_t ind) override;

        RobotRaconteur::RRArrayPtr<uint8_t> getf_composite_mesh_stl(uint32_t ind) override;

        com::robotraconteur::geometry::Transform getf_composite_mesh_transform(uint32_t ind) override;

    };


    class ScanningProcedure;
    class RunAlgorithms;
    class DeferredCapturePrepare;

    struct RRDeferredCapture
    {
        int32_t handle = -1;
        artec::sdk::base::TRef<artec::sdk::capturing::IFrame> frame;
        com::robotraconteur::geometry::shapes::MeshPtr mesh;
        RobotRaconteur::RRArrayPtr<uint8_t> mesh_stl_bytes;
    };

    using RRDeferredCapturePtr = boost::shared_ptr<RRDeferredCapture>;
    
    class ArtecScannerImpl : public experimental::artec_scanner::ArtecScanner_default_impl, 
        public RR_ENABLE_SHARED_FROM_THIS<ArtecScannerImpl>
    {

        private:
            artec::sdk::base::TRef<artec::sdk::capturing::IScanner> scanner = nullptr;
            artec::sdk::base::TRef<artec::sdk::capturing::IFrameProcessor> processor = nullptr;

            int32_t add_model(RRArtecModelPtr model);
                        
            int32_t handle_cnt = 100;
            std::map<int32_t,RRArtecModelPtr> models;
            std::map<int32_t,RRDeferredCapturePtr> deferred_captures;

            boost::mutex this_lock;

            boost::optional<boost::filesystem::path> save_path;

            void deferred_capture_to_iframemesh(const RRDeferredCapturePtr& deferred_capture, artec::sdk::base::IFrameMesh** frame_mesh);

            RRDeferredCapturePtr get_deferred_capture(int32_t deferred_capture_handle);

        public:
            friend class ScanningProcedure;
            friend class RunAlgorithms;
            friend class DeferredCapturePrepare;

            void Init(artec::sdk::capturing::IScanner* scanner);

            void set_save_path(boost::optional<boost::filesystem::path> save_path);

            com::robotraconteur::geometry::shapes::MeshPtr capture(RobotRaconteur::rr_bool with_texture) override;

            RobotRaconteur::RRArrayPtr<uint8_t> capture_stl() override;

            int32_t capture_deferred(RobotRaconteur::rr_bool with_texture) override;

            com::robotraconteur::geometry::shapes::MeshPtr getf_deferred_capture(int32_t deferred_capture_handle) override;

            RobotRaconteur::RRArrayPtr<uint8_t > getf_deferred_capture_stl(int32_t deferred_capture_handle) override;

            void deferred_capture_free(const RobotRaconteur::RRArrayPtr<int32_t>& deferred_capture_handle) override;

            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::DeferredCapturePrepareStatusPtr,void> 
                deferred_capture_prepare(const RobotRaconteur::RRArrayPtr<int32_t >& deferred_capture_handles) 
                override;

            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::DeferredCapturePrepareStatusPtr,void> 
                deferred_capture_prepare_stl(const RobotRaconteur::RRArrayPtr<int32_t >& deferred_capture_handles)
                override;


            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::ScanningProcedureStatusPtr,void>
                run_scanning_procedure(const experimental::artec_scanner::ScanningProcedureSettingsPtr& settings) 
                override;

            void model_free(int32_t model_handle) override;
            experimental::artec_scanner::ModelPtr get_models(int32_t model_handle) override;

            int32_t model_load(const std::string& project_name) override;

            void model_save(int32_t model_handle, const std::string& project_name) override;

            RobotRaconteur::RRValuePtr initialize_algorithm(int32_t input_model_handle, const std::string& algorithm) override;

            RobotRaconteur::GeneratorPtr<experimental::artec_scanner::RunAlgorithmsStatusPtr,void >
                run_algorithms(int32_t input_model_handle, const RobotRaconteur::RRListPtr<RobotRaconteur::RRValue>& algorithms) override;

            void free_all() override;

            virtual ~ArtecScannerImpl();
    };

    using ArtecScannerImplPtr = boost::shared_ptr<ArtecScannerImpl>;
    using ArtecScannerImplWeakPtr = boost::weak_ptr<ArtecScannerImpl>;

}
