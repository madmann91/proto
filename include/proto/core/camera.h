#ifndef MAU5_CORE_CAMERA_H
#define MAU5_CORE_CAMERA_H

#include "mau5/core/bbox.h"
#include "mau5/core/frustum.h"
#include "mau5/core/mat.h"

namespace mau5 {

template <typename T>
struct Camera {
    Camera()
        : Camera(Mat4x4<T>::identity(), Mat4x4<T>::identity())
    {}

    Camera(const Mat4x4<T>&, const Mat4x4<T>&);

    const Mat4x4<T>& view_proj() const { return view_proj_; }
    const Mat4x4<T>& proj() const { return proj_; }
    const Mat4x4<T>& view() const { return view_; }

    static Mat4x4<T> perspective_proj(T near, T far, T aspect_ratio, T horz_fov);
    static Mat4x4<T> perspective_proj(T left, T right, T bottom, T top, T near, T far);
    static Mat4x4<T> orthographic_proj(T left, T right, T bottom, T top, T near, T far);
    static Mat4x4<T> orthographic_proj(T near, T far, T aspect_ratio);
    static Mat4x4<T> view_matrix(const Vec3<T>& eye, const Vec3<T>& dir, const Vec3<T>& up);
    static Mat4x4<T> look_at(const Vec3<T>& eye, const Vec3<T>& at, const Vec3<T>& up);

private:
    Mat4x4<T> proj_;
    Mat4x4<T> view_;
    Mat4x4<T> view_proj_;
};

using Cameraf = Camera<float>;
using Camerad = Camera<double>;

} // namespace mau5

#endif
