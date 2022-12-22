#include "artec_scanner_util.h"

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/io/ObjIO.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>
#include <artec/sdk/base/io/PngIO.h>
namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;
namespace rr_image = com::robotraconteur::image;
namespace RR=RobotRaconteur;

namespace artec_scanner_robotraconteur_driver
{
    template<typename T>
    static RR::RRNamedArrayPtr<typename T> points3f_to_rr(asdk::TArrayPoint3F& points)
    {
        size_t points_count = static_cast<size_t>(points.size());
        auto rr_points = RR::AllocateEmptyRRNamedArray<T>(points_count);
        for (size_t i=0; i<points_count; i++)
        {
            auto& p = points[i];
            auto& rr_p = rr_points->at(i);
            rr_p.s.x = p.x;
            rr_p.s.y = p.y;
            rr_p.s.z = p.z;            
        }
        return rr_points;
    }

    static RR::RRNamedArrayPtr<rr_shapes::MeshTriangle> index_array_triangles_to_rr(asdk::TArrayIndexTriplet& ind_trip)
    {
        size_t count = static_cast<size_t>(ind_trip.size());
        auto rr_tri = RR::AllocateEmptyRRNamedArray<rr_shapes::MeshTriangle>(count);
        for (size_t i=0; i<count; i++)
        {
            auto& t = ind_trip[i];
            auto& rr_t = rr_tri->at(i);
            rr_t.s.v1 = t.x;
            rr_t.s.v2 = t.y;
            rr_t.s.v3 = t.z;            
        }
        return rr_tri;
    }

    RR::RRListPtr<rr_shapes::MeshTexture> get_mesh_texture_map(asdk::IFrameMesh* mesh)
    {
        TRef<asdk::IImage> img = mesh->getImage();
        TRef<asdk::IArrayUVCoordinates> uv = mesh->getUVCoordinates();
        if (img == nullptr || uv == nullptr)
        {
            return nullptr;
        }
        TRef<asdk::IBlob> img_blob;
        auto ec = artec::sdk::base::io::savePngImageToBlob(&img_blob, img);
        if (ec != asdk::ErrorCode::ErrorCode_OK)
        {
            throw RR::OperationFailedException("Could not convert image to PNG");
        }

        auto compressed_image_bytes = RR::AttachRRArrayCopy<uint8_t>(static_cast<uint8_t*>(img_blob->getPointer()), 
            img_blob->getSize());

        auto compressed_image_info = rr_image::ImageInfoPtr(new rr_image::ImageInfo());
        compressed_image_info->height = img->getHeight();
        compressed_image_info->width = img->getWidth();
        compressed_image_info->step = img->getPitch();
        compressed_image_info->encoding = rr_image::ImageEncoding::compressed;

        auto image = rr_image::CompressedImagePtr(new rr_image::CompressedImage());
        image->data = compressed_image_bytes;
        image->image_info = compressed_image_info;

        auto rr_uv = RR::AllocateEmptyRRNamedArray<rr_geom::Vector2>((size_t)uv->getSize());
        auto uv_coords = uv->getPointer();
        for (size_t i=0; i<rr_uv->size(); i++)
        {
            auto& rr_p = rr_uv->at(i);
            auto& p = uv_coords[i];
            rr_p.s.x = p.u;
            rr_p.s.y = p.v;
        }

        auto rr_tex = rr_shapes::MeshTexturePtr(new rr_shapes::MeshTexture());
        rr_tex->image = image;
        rr_tex->uvs = rr_uv;

        auto ret = RR::AllocateEmptyRRList<rr_shapes::MeshTexture>();
        ret->push_back(rr_tex);
        return ret;
    }
    com::robotraconteur::geometry::shapes::MeshPtr ConvertArtecMeshToRR(artec::sdk::base::IFrameMesh* mesh)
    {
        auto ret = rr_shapes::MeshPtr(new rr_shapes::Mesh());
        
        asdk::TArrayPoint3F points = mesh->getPoints();
        ret->vertices = points3f_to_rr<rr_geom::Point>(points);
        asdk::TArrayIndexTriplet triangles = mesh->getTriangles();
        ret->triangles = index_array_triangles_to_rr(triangles);
        mesh->calculate( asdk::CM_Normals );
        asdk::TArrayPoint3F points_normals  = mesh->getPointsNormals();
        ret->normals = points3f_to_rr<rr_geom::Vector3>(points_normals);
        ret->colors = RR::AllocateEmptyRRNamedArray<com::robotraconteur::color::ColorRGB>(0);

        ret->textures = get_mesh_texture_map(mesh);

        return ret;
    }
}