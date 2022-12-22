#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/base/IFrameMesh.h>
#include <com__robotraconteur__geometry__shapes.h>

namespace artec_scanner_robotraconteur_driver
{
    com::robotraconteur::geometry::shapes::MeshPtr ConvertArtecMeshToRR(artec::sdk::base::IFrameMesh* mesh);
}