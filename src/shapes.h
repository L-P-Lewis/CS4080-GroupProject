#ifndef KINSOLVER_SHAPES_H_
#define KINSOLVER_SHAPES_H_

#include <vector>
#include <tuple>
#include <cmath>

namespace KinSolver {

    struct AABB {
        double X, Y, Width, Height;
    };

    class Vector2 {
    public:
        double X;
        double Y;

        Vector2 operator*(double scalar) const;
        Vector2 operator+(const Vector2& Other) const;
        Vector2 operator-(const Vector2& Other) const;
        bool operator<(const Vector2& Other) const;
        bool operator==(const Vector2& Other) const;

        Vector2 Normalized() const;
        double Length() const;

        Vector2() : X(0.0), Y(0.0) {}
        Vector2(double X, double Y) : X(X), Y(Y) {}

        static double Vector2Dot(const Vector2& A, const Vector2& B);
    };

    class Shape {
    public:
        Vector2 Position;
        Shape() : Position(Vector2()) {}
        virtual std::vector<Vector2> GetSeparationAxes() = 0;
        virtual std::tuple<double, double> ProjectShape(Vector2 Axis) = 0;
        virtual AABB GetSweptAABB(Vector2 Movement) = 0;
    };

    class Polygon : public Shape {
    public:
        std::vector<Vector2> Points;

        std::tuple<double, double> ProjectShape(Vector2 Axis) override;
        std::vector<Vector2> GetSeparationAxes() override;
        AABB GetSweptAABB(Vector2 Movement) override;
    };

}

#endif
