#include "artec_scanner_util.h"

#include <artec/sdk/algorithms/IAlgorithm.h>
#include <artec/sdk/base/ScannerType.h>

#pragma once

namespace artec_scanner_robotraconteur_driver
{
    void create_fast_fusion_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::FastFusionAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_poisson_fusion_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::PoissonFusionAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_serial_registration_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::SerialRegistrationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_auto_align_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::AutoAlignAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_global_registration_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::GlobalRegistrationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_texturization_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::TexturizationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_small_objects_filter_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::SmallObjectsFilterAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_mesh_simplification_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::MeshSimplificationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_fast_mesh_simplification_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::FastMeshSimplificationAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_loop_closure_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::LoopClosureAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

    void create_outliers_removal_algorithm(artec::sdk::algorithms::IAlgorithm** alg, 
        const experimental::artec_scanner::OutliersRemovalAlgorithmPtr& settings, 
        artec::sdk::base::ScannerType scanner_type);

}