#include "artec_scanner_algorithm_util.h"

#include <artec/sdk/algorithms/Algorithms.h>

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

}