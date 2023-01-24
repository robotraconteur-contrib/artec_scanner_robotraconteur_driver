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
#include <artec/sdk/project/IProject.h>
#include <artec/sdk/project/EntryInfo.h>
#include <artec/sdk/project/ProjectSettings.h>
#include <artec/sdk/project/ProjectLoaderSettings.h>
#include <artec/sdk/project/ProjectSaverSettings.h>

#include "artec_scanner_util.h"
#include "artec_scanning_procedure.h"

#include <boost/filesystem.hpp>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
    using namespace artec::sdk::project;
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

    void ArtecScannerImpl::set_save_path(boost::optional<boost::filesystem::path> save_path)
    {
        if (!save_path)
        {
            this->save_path = save_path;
            RR_ARTEC_LOG_INFO("Project save path cleared");
            return;
        }
        if (!boost::filesystem::is_directory(*save_path))
        {
            RR_ARTEC_LOG_ERROR("Project save path is not a directory: " << *save_path);
            throw RR::InvalidArgumentException("Project save path is not a directory");
        }

        this->save_path = save_path;
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

    int32_t ArtecScannerImpl::load_model(const std::string& project_name)
    {
        if (!save_path)
        {
            RR_ARTEC_LOG_ERROR("load_model called but project save path not specified")
            throw RR::InvalidOperationException("Project save path not specified");
        }
        boost::regex r_project_name("^[\\w\\-]+$");
        if(!boost::regex_match(project_name,r_project_name))
        {
            RR_ARTEC_LOG_ERROR("Invalid project name specified: " << project_name);
            throw RR::InvalidArgumentException("Invalid project name");
        }

        
        auto file_path = ((*save_path) / project_name / (project_name + ".a3d"));
        RR_ARTEC_LOG_INFO("Begin load model from file " << file_path);
        asdk::TRef<asdk::IProject> project;

        RR_CALL_ARTEC(asdk::openProject(&project, file_path.c_str()), "Could not open project");

        int num_entries = project->getEntryCount();
        asdk::TRef<asdk::IArrayUuid> uuids;
        RR_CALL_ARTEC(asdk::createArrayUuid(&uuids, num_entries), "Could not allocate project load uuids");
        for (int i=0; i<num_entries; i++)
        {
            asdk::EntryInfo entry;
            RR_CALL_ARTEC(project->getEntry(i, &entry), "Could not get entry info");
            uuids->setElement(i, entry.uuid);       
        }

        asdk::ProjectLoaderSettings loader_settings;
        loader_settings.entryList = uuids;
        asdk::TRef<asdk::IJob> loader;
        RR_CALL_ARTEC(project->createLoader(&loader, &loader_settings), "Could not create loader");
        auto model = RR_MAKE_SHARED<RRArtecModel>();
        asdk::AlgorithmWorkset load_workset = {nullptr, model->model, nullptr, 0};
        RR_CALL_ARTEC(asdk::executeJob(loader, &load_workset), "Error loading model");

        int32_t model_handle = add_model(model);
        RR_ARTEC_LOG_INFO("Loaded model: " << model_handle << " from file " << file_path);
        return model_handle;
    }

    void ArtecScannerImpl::save_model(const std::string& project_name, int32_t model_handle)
    {
        if (!save_path)
        {
            RR_ARTEC_LOG_ERROR("load_model called but project save path not specified")
            throw RR::InvalidOperationException("Project save path not specified");
        }
        boost::regex r_project_name("^[\\w\\-]+$");
        if(!boost::regex_match(project_name,r_project_name))
        {
            RR_ARTEC_LOG_ERROR("Invalid project name specified: " << project_name);
            throw RR::InvalidArgumentException("Invalid project name");
        }

        auto project_dir = *save_path / project_name;
        if (boost::filesystem::exists(project_dir))
        {
            RR_ARTEC_LOG_ERROR("Project directory already exists: " << project_dir);
            throw RR::InvalidArgumentException("Project name already exstis");
        }

        RRArtecModelPtr model = RR_DYNAMIC_POINTER_CAST<RRArtecModel>(get_models(model_handle));

        asdk::ProjectSaverSettings save_settings;
        
        boost::filesystem::create_directory(project_dir);
        auto file_path = ((project_dir) / (project_name + ".a3d"));
        save_settings.path = file_path.c_str();
        RR_ARTEC_LOG_INFO("Begin save model: " << model_handle << " to file " << file_path);
        RR_CALL_ARTEC(asdk::generateUuid(&save_settings.projectId), "Error generating project UUID");
        asdk::ProjectSettings project_settings;
        project_settings.path = file_path.c_str();
        asdk::TRef<asdk::IProject> project;
        asdk::createNewProject(&project, &project_settings);
        asdk::TRef<asdk::IJob> saver;
        RR_CALL_ARTEC(project->createSaver(&saver, &save_settings), "Error creating project saver");
        asdk::AlgorithmWorkset save_workset = {model->model, nullptr, nullptr, 0};
        RR_CALL_ARTEC(asdk::executeJob(saver, &save_workset), "Error saving model");
        RR_ARTEC_LOG_INFO("Saved model: " << model_handle << " to file " << file_path);
    }

    RR::GeneratorPtr<rr_artec::RunAlgorithmsStatusPtr,void >
        ArtecScannerImpl::run_algorithms(const RR::RRListPtr<RR::RRValue>& algorithms, int32_t input_model_handle)
    {
        throw RR::NotImplementedException("Not implemented");
    }

}