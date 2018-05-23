#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace pydrake {

template <typename T>
class PyTrajectory : public py::wrapper<trajectories::Trajectory<T>> {
 public:
  using Base = py::wrapper<trajectories::Trajectory<T>>;
  using Base::Base;

  std::unique_ptr<trajectories::Trajectory<T>> Clone() const override {
    PYBIND11_OVERLOAD_PURE(
        std::unique_ptr<trajectories::Trajectory<T>>, trajectories::Trajectory<T>, "_Clone");
    return Base::Clone();
  }

  MatrixX<T> value(double t) const override {
    PYBIND11_OVERLOAD_PURE(
        MatrixX<T>, trajectories::Trajectory<T>, "_value", t);
    return Base::value(t);
  }

  std::unique_ptr<trajectories::Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override {
    PYBIND11_OVERLOAD_PURE(
        std::unique_ptr<trajectories::Trajectory<T>>, trajectories::Trajectory<T>, 
        "_MakeDerivative", derivative_order);
    return Base::MakeDerivative(derivative_order);
  }

  Eigen::Index rows() const override {
    PYBIND11_OVERLOAD_PURE(
        Eigen::Index, trajectories::Trajectory<T>, "_rows");
    return Base::rows();
  }

  Eigen::Index cols() const override {
    PYBIND11_OVERLOAD_PURE(
        Eigen::Index, trajectories::Trajectory<T>, "_cols");
    return Base::cols();
  }

  double start_time() const override {
    PYBIND11_OVERLOAD_PURE(
        double, trajectories::Trajectory<T>, "_start_time");
    return Base::start_time();
  }

  double end_time() const override {
    PYBIND11_OVERLOAD_PURE(
        double, trajectories::Trajectory<T>, "_end_time");
    return Base::end_time();
  }
};

PYBIND11_MODULE(trajectories, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::trajectories;

  using T = double;

  py::class_<Trajectory<T>, PyTrajectory<T>>(
      m, "Trajectory"
  );

  py::class_<PiecewiseTrajectory<T>, Trajectory<T>>(
      m, "PiecewiseTrajectory")
      .def("get_number_of_segments",
           &PiecewiseTrajectory<T>::get_number_of_segments)
      .def("start_time", overload_cast_explicit<double, int>(
                             &PiecewiseTrajectory<T>::start_time))
      .def("end_time", overload_cast_explicit<double, int>(
                           &PiecewiseTrajectory<T>::end_time))
      .def("duration", &PiecewiseTrajectory<T>::duration)
      .def("start_time",
           overload_cast_explicit<double>(&PiecewiseTrajectory<T>::start_time))
      .def("end_time",
           overload_cast_explicit<double>(&PiecewiseTrajectory<T>::end_time))
      .def("get_segment_index", &PiecewiseTrajectory<T>::get_segment_index)
      .def("get_segment_times", &PiecewiseTrajectory<T>::get_segment_times);

  py::class_<PiecewisePolynomial<T>, PiecewiseTrajectory<T>>(
      m, "PiecewisePolynomial")
      .def(py::init<>())
      .def(py::init<const Eigen::Ref<const MatrixX<T>>&>())
      .def_static("ZeroOrderHold",
                  py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const MatrixX<T>>&>(
                      &PiecewisePolynomial<T>::ZeroOrderHold))
      .def_static("FirstOrderHold",
                  py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const MatrixX<T>>&>(
                      &PiecewisePolynomial<T>::FirstOrderHold))
      .def_static("Pchip",
                  py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const MatrixX<T>>&, bool>(
                      &PiecewisePolynomial<T>::Pchip))
      .def_static("Cubic",
                  py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const MatrixX<T>>&,
                                    const Eigen::Ref<const VectorX<T>>&,
                                    const Eigen::Ref<const VectorX<T>>&>(
                      &PiecewisePolynomial<T>::Cubic),
                  py::arg("breaks"), py::arg("knots"),
                  py::arg("knot_dot_start"), py::arg("knot_dot_end"))
      .def_static("Cubic",
                  py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const MatrixX<T>>&,
                                    const Eigen::Ref<const MatrixX<T>>&>(
                      &PiecewisePolynomial<T>::Cubic),
                  py::arg("breaks"), py::arg("knots"), py::arg("knots_dot"))
      .def_static("Cubic",
                  py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                    const Eigen::Ref<const MatrixX<T>>&>(
                      &PiecewisePolynomial<T>::Cubic),
                  py::arg("breaks"), py::arg("knots"))
      .def("value", &PiecewisePolynomial<T>::value)
      .def("rows", &PiecewisePolynomial<T>::rows)
      .def("cols", &PiecewisePolynomial<T>::cols);
}

}  // namespace pydrake
}  // namespace drake
