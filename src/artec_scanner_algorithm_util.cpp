#include "artec_scanner_algorithm_util.h"

#include <artec/sdk/algorithms/Algorithms.h>
#include <artec/sdk/base/IScan.h>

#include "artec_scanner_impl.h"

#define RR_ARTEC_PREFIX "experimental.artec_scanner."

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::algorithms;
};
using asdk::TRef;

namespace rr_artec = experimental::artec_scanner;


namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_image = com::robotraconteur::image;
namespace RR=RobotRaconteur;

namespace artec_scanner_robotraconteur_driver
{
void create_fast_fusion_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
    const experimental::artec_scanner::FastFusionAlgorithmPtr& settings, 
    artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::FastFusionSettings s;
    s.generateNormals = settings->generate_normals.value != 0;
    s.radius = settings->radius;
    s.resolution = settings->resolution;
    s.scannerType = scanner_type;

    RR_CALL_ARTEC(asdk::createFastFusionAlgorithm(alg, &s), "Could not create FastFusionAlgorithm");
}

void create_poisson_fusion_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
    const experimental::artec_scanner::PoissonFusionAlgorithmPtr& settings, 
    artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::PoissonFusionSettings s;
    s.fusionType = (asdk::PoissonFusionType)settings->fusion_type;
    s.fillType = (asdk::FillHolesType)settings->fill_type;
    s.resolution = settings->resolution;
    s.maxHoleRadius = settings->max_hole_radius;
    s.removeTargets = settings->remove_targets.value != 0;
    s.targetInnerSize = settings->target_inner_size;
    s.targetOuterSize = settings->target_outer_size;
    s.generateNormals = settings->generate_normals.value != 0;
    s.scannerType = scanner_type;

    RR_CALL_ARTEC(asdk::createPoissonFusionAlgorithm(alg, &s), "Could not create PoissonFusionAlgorithm");
}

void create_serial_registration_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
    const experimental::artec_scanner::SerialRegistrationAlgorithmPtr& settings, 
    artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::SerialRegistrationSettings s;
    s.registrationType = (asdk::SerialRegistrationType)settings->registration_type;
    s.scannerType = scanner_type;

    RR_CALL_ARTEC(asdk::createSerialRegistrationAlgorithm(alg, &s), "Could not create SerialRegistrationAlgorithm");
}

void create_auto_align_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
    const experimental::artec_scanner::AutoAlignAlgorithmPtr& settings, 
    artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::AutoAlignSettings s;
    s.scannerType = scanner_type;

    RR_CALL_ARTEC(asdk::createAutoalignAlgorithm(alg, &s), "Could not create AutoAlignAlgorithm");
}

void create_global_registration_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
    const experimental::artec_scanner::GlobalRegistrationAlgorithmPtr& settings, 
    artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::GlobalRegistrationSettings s;
    s.registrationType = (asdk::GlobalRegistrationType)settings->registration_type;
    s.scannerType = scanner_type;

    RR_CALL_ARTEC(asdk::createGlobalRegistrationAlgorithm(alg, &s), "Could not create GlobalRegistrationAlgorithm");
}

void create_texturization_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
    const experimental::artec_scanner::TexturizationAlgorithmPtr& settings, 
    artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::TexturizationSettings s;
    s.scannerType = scanner_type;
    s.texturizeType = (asdk::TexturizeType)settings->texturize_type;
    s.texturizeResolution = (asdk::TexturizeResolution)settings->texturize_resolution;
    s.enableBackgroundSegmentation = settings->enable_background_segmentation.value != 0;
    s.enableAmbientLightingCompensation = settings->enable_ambient_lighting_compensation.value != 0;
    s.atlasUnfoldingPolygonLimit = settings->atlas_unfolding_polygon_limit;
    s.enableTextureInpainting = settings->enable_texture_inpainting.value != 0;
    s.useTextureNormalization = settings->use_texture_normalization.value != 0;
    s.inputFilterType = (asdk::InputFilter)settings->input_filter_type;

    RR_CALL_ARTEC(asdk::createTexturizationAlgorithm(alg, &s), "Could not create TexturizationAlgorithm");
}

void create_small_objects_filter_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::SmallObjectsFilterAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::SmallObjectsFilterSettings s;
    s.filterType = (asdk::SmallObjectsFilterType)settings->filter_type;
    s.filterThreshold = settings->filter_threshold;
    s.scannerType = scanner_type;

    RR_CALL_ARTEC(asdk::createSmallObjectsFilterAlgorithm(alg, &s), "Could not create SmallObjectsFilterAlgorithm");
}

void create_mesh_simplification_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::MeshSimplificationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::MeshSimplificationSettings s;
    s.scannerType = scanner_type;
    s.simplifyType = (asdk::SimplifyType)settings->simplify_type;
    s.simplifyMetrics = (asdk::SimplifyMetric)settings->simplify_metrics;
    s.triangleNumber = settings->triangle_number;
    s.keepBoundary = settings->keep_boundary.value != 0;
    s.angleThreshold = settings->angle_threshold;
    s.remeshEdgeThreshold = settings->remesh_edge_threshold;
    s.error = settings->error;

    RR_CALL_ARTEC(asdk::createMeshSimplificationAlgorithm(alg, &s), "Could not create MeshSimplificationAlgorithm");
}


void create_fast_mesh_simplification_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::FastMeshSimplificationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::FastMeshSimplificationSettings s;
    s.scannerType = scanner_type;    
    s.triangleNumber = settings->triangle_number;
    s.keepBoundary = settings->keep_boundary.value != 0;
    s.enableAdditionalCriteria = settings->enable_additional_criteria.value != 0;
    s.enableDistanceThreshold = settings->enable_distance_threshold.value != 0;
    s.distanceThreshold = settings->distance_threshold;
    s.enableAngleThreshold = settings->enable_angle_threshold.value != 0;
    s.angleThreshold = settings->angle_threshold;
    s.enableAspectRatioThreshold = settings->enable_aspect_ratio_threshold != 0;
    s.aspectRatioThreshold = settings->aspect_ratio_threshold;    

    RR_CALL_ARTEC(asdk::createFastMeshSimplificationAlgorithm(alg, &s), "Could not create FastMeshSimplificationAlgorithm");
}

void create_loop_closure_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::LoopClosureAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::LoopClosureSettings s;
    s.scannerType = scanner_type;    

    RR_CALL_ARTEC(asdk::createLoopClosureAlgorithm(alg, &s), "Could not create LoopClosureAlgorithm");
}

void create_outliers_removal_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::OutliersRemovalAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type)
{
    RR_NULL_CHECK(settings);
    asdk::OutliersRemovalSettings s;
    s.scannerType = scanner_type;
    s.standardDeviationMultiplier = settings->standard_deviation_multiplier;
    s.resolution = settings->resolution;

    RR_CALL_ARTEC(asdk::createOutliersRemovalAlgorithm(alg, &s), "Could not create OutlierRemovalsAlgorithm");
}

RobotRaconteur::RRValuePtr util_initialize_algorithm(boost::shared_ptr<RRArtecModel> model, const std::string& algorithm)
{
    if (model->model->getSize() <= 0)
    {
        RR_ARTEC_LOG_ERROR("Model passed to initailez_algorithm does not contain any scans")
        throw RR::InvalidArgumentException("Model passed to initialize_algorithm does not contain any scans");
    }

    auto scanner_type = model->model->getElement(0)->getScannerType();

    if (algorithm == "AutoAlignAlgorithm" || algorithm == RR_ARTEC_PREFIX "AutoAlignAlgorithm" )
    {
        return rr_artec::AutoAlignAlgorithmPtr(new rr_artec::AutoAlignAlgorithm());
    }

    if (algorithm == "FastFusionAlgorithm" || algorithm == RR_ARTEC_PREFIX "FastFusionAlgorithm" )
    {
        asdk::FastFusionSettings a;
        asdk::initializeFastFusionSettings(&a, scanner_type);
        auto rr = rr_artec::FastFusionAlgorithmPtr(new rr_artec::FastFusionAlgorithm());
        rr->resolution = a.resolution;
        rr->radius = a.radius;
        rr->generate_normals.value = a.generateNormals ? 1 : 0;
        return rr;
    }

    if (algorithm == "FastMeshSimplificationAlgorithm" || algorithm == RR_ARTEC_PREFIX "FastMeshSimplificationAlgorithm")
    {
        asdk::FastMeshSimplificationSettings a;
        asdk::initializeFastMeshSimplificationSettings(&a, scanner_type);
        auto rr = rr_artec::FastMeshSimplificationAlgorithmPtr(new rr_artec::FastMeshSimplificationAlgorithm());
        rr->triangle_number = a.triangleNumber;
        rr-> keep_boundary.value = a.keepBoundary ? 1 : 0;
        rr->enable_additional_criteria = a.enableAdditionalCriteria ? 1 : 0;
        rr->enable_distance_threshold = a.enableDistanceThreshold ? 1 : 0;
        rr->distance_threshold = a.distanceThreshold;
        rr->enable_angle_threshold = a.enableAngleThreshold ? 1 : 0;
        rr->angle_threshold = a.angleThreshold;
        rr->enable_aspect_ratio_threshold = a.enableAspectRatioThreshold ? 1 : 0;
        rr->aspect_ratio_threshold = a.enableAspectRatioThreshold;
        return rr;
    }

    if (algorithm == "GlobalRegistrationAlgorithm" || algorithm == RR_ARTEC_PREFIX "GlobalRegistrationAlgorithm" )
    {
        auto rr = rr_artec::GlobalRegistrationAlgorithmPtr(new rr_artec::GlobalRegistrationAlgorithm());
        rr->registration_type = rr_artec::GlobalRegistrationType::geometry;
        return rr;
    }

    if (algorithm == "LoopClosureAlgorithm" || algorithm == RR_ARTEC_PREFIX "LoopClosureAlgorithm" )
    {
        return rr_artec::LoopClosureAlgorithmPtr(new rr_artec::LoopClosureAlgorithm());
    }

    if (algorithm == "MeshSimplificationAlgorithm" || algorithm == RR_ARTEC_PREFIX "MeshSimplificationAlgorithm" )
    {
        asdk::MeshSimplificationSettings a;
        asdk::initializeMeshSimplificationSettings(&a, scanner_type, asdk::SimplifyType_Accuracy );
        auto rr = rr_artec::MeshSimplificationAlgorithmPtr(new rr_artec::MeshSimplificationAlgorithm());
        rr->simplify_type = (rr_artec::SimplifyType::SimplifyType)a.simplifyType;
        rr->simplify_metrics = (rr_artec::SimplifyMetric::SimplifyMetric)a.simplifyMetrics;
        rr->triangle_number = a.triangleNumber;
        rr->keep_boundary = a.keepBoundary ? 1 : 0;
        rr->angle_threshold = a.angleThreshold;
        rr->remesh_edge_threshold = a.remeshEdgeThreshold;
        rr->error = a.error;
        return rr;
    }

    if (algorithm == "OutliersRemovalAlgorithm" || algorithm == RR_ARTEC_PREFIX "OutliersRemovalAlgorithm" )
    {
        asdk::OutliersRemovalSettings a;
        asdk::initializeOutliersRemovalSettings(&a, scanner_type);
        auto rr = rr_artec::OutliersRemovalAlgorithmPtr(new rr_artec::OutliersRemovalAlgorithm());
        rr->standard_deviation_multiplier = a.standardDeviationMultiplier;
        rr->resolution = a.resolution;
        return rr;
    }

    if (algorithm == "PoissonFusionAlgorithm" || algorithm == RR_ARTEC_PREFIX "PoissonFusionAlgorithm" )
    {
        asdk::PoissonFusionSettings a;
        asdk::initializePoissonFusionSettings(&a, scanner_type);
        auto rr = rr_artec::PoissonFusionAlgorithmPtr(new rr_artec::PoissonFusionAlgorithm());
        rr->fusion_type = (rr_artec::PoissonFusionType::PoissonFusionType)a.fusionType;
        rr->fill_type = (rr_artec::FillHolesType::FillHolesType)a.fillType;
        rr->resolution = a.resolution;
        rr->max_hole_radius = a.maxHoleRadius;
        rr->remove_targets.value = a.removeTargets ? 1 : 0;
        rr->target_inner_size = a.targetInnerSize;
        rr->target_outer_size = a.targetOuterSize;
        rr->generate_normals = a.generateNormals ? 1 : 0;
        rr->input_filter_type = (rr_artec::InputFilter::InputFilter)a.inputFilterType;
        return rr;
    }

    if (algorithm == "SerialRegistrationAlgorithm" || algorithm == RR_ARTEC_PREFIX "SerialRegistrationAlgorithm" )
    {       
        auto rr = rr_artec::SerialRegistrationAlgorithmPtr(new rr_artec::SerialRegistrationAlgorithm());
        rr->registration_type = (rr_artec::SerialRegistrationType::rough);        
        return rr;
    }

    if (algorithm == "SmallObjectsFilterAlgorithm" || algorithm == RR_ARTEC_PREFIX "SmallObjectsFilterAlgorithm" )
    {
        asdk::SmallObjectsFilterSettings a;
        asdk::initializeSmallObjectsFilterSettings(&a, scanner_type);
        auto rr = rr_artec::SmallObjectsFilterAlgorithmPtr(new rr_artec::SmallObjectsFilterAlgorithm());
        rr->filter_type = (rr_artec::SmallObjectsFilterType::SmallObjectsFilterType)a.filterType;
        rr->filter_threshold = a.filterThreshold;
        return rr;
    }

    if (algorithm == "TexturizationAlgorithm" || algorithm == RR_ARTEC_PREFIX "TexturizationAlgorithm" )
    {
        asdk::TexturizationSettings a;
        asdk::initializeTexturizationSettings(&a, scanner_type);
        auto rr = rr_artec::TexturizationAlgorithmPtr(new rr_artec::TexturizationAlgorithm());
        rr->texturize_type = (rr_artec::TexturizeType::TexturizeType)a.texturizeType;
        rr->texturize_resolution = (rr_artec::TexturizeResolution::TexturizeResolution)a.texturizeResolution;
        rr->enable_background_segmentation.value = a.enableBackgroundSegmentation ? 1 : 0;
        rr->enable_ambient_lighting_compensation.value = a.enableAmbientLightingCompensation ? 1 : 0;
        rr->atlas_unfolding_polygon_limit = a.atlasUnfoldingPolygonLimit;
        rr->enable_texture_inpainting.value = a.enableTextureInpainting ? 1 : 0;
        rr->use_texture_normalization = a.useTextureNormalization ? 1 : 0;
        rr->input_filter_type = (rr_artec::InputFilter::InputFilter)a.inputFilterType;
        return rr;
    }

    RR_ARTEC_LOG_ERROR("Invalid algorithm requested: " << algorithm);
    throw RR::InvalidArgumentException("Invalid algorithm requested");
}

}