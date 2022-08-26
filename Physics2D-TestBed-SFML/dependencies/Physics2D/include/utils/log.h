#ifndef UTILS_LOG_H
#define UTILS_LOG_H
#include "spdlog/spdlog.h"
#include "spdlog/fmt/fmt.h"
#include "dependencies/Physics2D/include/math/linear/linear.h"

template<> struct fmt::formatter<Physics2D::Vec2> : formatter<float> {
    template <typename FormatContext>
    auto format(const Physics2D::Vec2& vec, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "Vec2({:.3f}, {:.3f})", vec.x, vec.y);
    }
};
template<> struct fmt::formatter<Physics2D::Vec3> : formatter<float> {
    template <typename FormatContext>
    auto format(const Physics2D::Vec3& vec, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "Vec3({:.3f}, {:.3f}, {:.3f})", vec.x, vec.y, vec.z);
    }
};
template<> struct fmt::formatter<Physics2D::Vec4> : formatter<float> {
    template <typename FormatContext>
    auto format(const Physics2D::Vec4& vec, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "Vec4({:.3f}, {:.3f}, {:.3f}, {:.3f})", vec.x, vec.y, vec.z, vec.w);
    }
};
template<> struct fmt::formatter<Physics2D::Mat2> : formatter<float> {
    template <typename FormatContext>
    auto format(Physics2D::Mat2& mat, FormatContext& ctx) {
        return fmt::format_to(ctx.out(),
            "\n[{:.3f}, {:.3f}]"
            "\n[{:.3f}, {:.3f}]",
            mat[0][0], mat[1][0],
            mat[0][1], mat[1][1]);
    }
};
template<> struct fmt::formatter<Physics2D::Mat3> : formatter<float> {
    template <typename FormatContext>
    auto format(const Physics2D::Mat3& mat, FormatContext& ctx) {
        return fmt::format_to(ctx.out(),
            "\n[{:.3f}, {:.3f}, {:.3f}]"
            "\n[{:.3f}, {:.3f}, {:.3f}]"
            "\n[{:.3f}, {:.3f}, {:.3f}]",
            mat.e00(), mat.e01(), mat.e02(),
            mat.e10(), mat.e11(), mat.e12(),
            mat.e20(), mat.e21(), mat.e22());
    }
};
template<> struct fmt::formatter<Physics2D::Mat4> : formatter<float> {
    template <typename FormatContext>
    auto format(const Physics2D::Mat4& mat, FormatContext& ctx) {
        return fmt::format_to(ctx.out(),
            "\n[{:.3f}, {:.3f}, {:.3f}, {:.3f}]"
            "\n[{:.3f}, {:.3f}, {:.3f}, {:.3f}]"
            "\n[{:.3f}, {:.3f}, {:.3f}, {:.3f}]"
            "\n[{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
            mat.e00(), mat.e01(), mat.e02(), mat.e03(),
            mat.e10(), mat.e11(), mat.e12(), mat.e13(),
            mat.e20(), mat.e21(), mat.e22(), mat.e23(),
            mat.e30(), mat.e31(), mat.e32(), mat.e33());
    }
};
#endif