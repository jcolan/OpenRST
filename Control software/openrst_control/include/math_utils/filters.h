#ifndef FILTERS_HEADER_GUARD
#define FILTERS_HEADER_GUARD

#include <Eigen/Dense>

using namespace Eigen;

namespace filters
{

class SMA
{

public:
  SMA(int n, int m) : N_elements_(n), order_(m)
  {
    el_mat_          = MatrixXd::Zero(N_elements_, order_);
    flag_initialized = false;
  }

  VectorXd get_sma()
  {
    VectorXd sma = VectorXd::Zero(N_elements_);
    for (int i = 0; i < N_elements_; i++) {
      sma[i] = el_mat_.row(i).sum() / order_;
    }
    return sma;
  }

  void shift(int n)
  {
    MatrixXd el_mat_tmp;
    el_mat_tmp = MatrixXd::Map(&el_mat_(0, n), N_elements_, (order_ - n));
    MatrixXd::Map(&el_mat_(0, 0), N_elements_, (order_ - n)) = el_mat_tmp;
    MatrixXd::Map(&el_mat_(0, order_ - n), N_elements_, n) =
        MatrixXd::Zero(N_elements_, n);
  }

  void add(VectorXd new_vec)
  {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&el_mat_(0, order_ - 1), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM(
          "avg_filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" Mat Filter: " << el_mat_);
  }

  void init_full(VectorXd i_vector)
  {
    for (int i = 0; i < order_; i++) {
      shift(1);
      add(i_vector);
    }
    flag_initialized = true;
    // ROS_INFO_STREAM(" Mat Initi: " << el_mat_);
  }

  VectorXd update(VectorXd new_vec)
  {
    if (!flag_initialized) {
      init_full(new_vec);
    }
    shift(1);
    add(new_vec);
    VectorXd out_filtered = get_sma();
    return out_filtered;
  }

private:
  int      N_elements_;
  int      order_;
  MatrixXd el_mat_;
  bool     flag_initialized;
}; // class end

class IIR
{

public:
  // [n] Elements to filter
  // [m] Filter Order
  // [b_coef] Filter denominator (size: (m+1)x1)
  // [a_coed] Filter Numerator  (size: mx1)
  IIR(int n, int m, VectorXd b_coef, VectorXd a_coef)
      : N_elements_(n), order_(m), b_coef_(b_coef), a_coef_(a_coef)
  {
    Xmat_            = MatrixXd::Zero(N_elements_, order_ + 1);
    Ymat_            = MatrixXd::Zero(N_elements_, order_);
    flag_initialized = false;
  }

  VectorXd get_iir()
  {
    VectorXd iir = Xmat_ * b_coef_ - Ymat_ * a_coef_;
    // for (int i = 0; i < N_elements_; i++) {
    //   iir[i] = el_mat_.row(i).sum() / order_;
    // }
    return iir;
  }

  void shiftX(int n)
  {
    // ROS_INFO_STREAM("X_mat_before: " << Xmat_);
    MatrixXd X_mat_tmp;
    X_mat_tmp = MatrixXd::Map(&Xmat_(0, 0), N_elements_, ((order_ + 1) - n));
    MatrixXd::Map(&Xmat_(0, n), N_elements_, ((order_ + 1) - n)) = X_mat_tmp;
    // ROS_INFO_STREAM("X_mat_after: " << Xmat_);

    MatrixXd::Map(&Xmat_(0, 0), N_elements_, n) =
        MatrixXd::Zero(N_elements_, n);
  }

  void shiftY(int n)
  {
    MatrixXd Ymat_tmp;
    Ymat_tmp = MatrixXd::Map(&Ymat_(0, 0), N_elements_, (order_ - n));
    MatrixXd::Map(&Ymat_(0, n), N_elements_, (order_ - n)) = Ymat_tmp;

    MatrixXd::Map(&Ymat_(0, 0), N_elements_, n) =
        MatrixXd::Zero(N_elements_, n);
  }

  void addX(VectorXd new_vec)
  {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&Xmat_(0, 0), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM(
          "filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" X Mat Filter: " << Xmat_);
  }

  void addY(VectorXd new_vec)
  {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&Ymat_(0, 0), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM(
          "filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" Y Mat Filter: " << Ymat_);
  }

  void init_full(VectorXd i_vector)
  {
    for (int i = 0; i < order_ + 1; i++) {
      shiftX(1);
      addX(i_vector);
      if (i < order_) {
        shiftY(1);
        // addY(get_iir());
        addY(i_vector);
      }
    }
    flag_initialized = true;

    // ROS_INFO_STREAM(" Mat Initi: " << el_mat_);
  }

  VectorXd update(VectorXd new_vec)
  {
    if (!flag_initialized) {
      init_full(new_vec);
    }
    // ROS_INFO_STREAM("Y Mat:" << Ymat_);
    // ROS_INFO_STREAM("X Mat:" << Xmat_);
    addX(new_vec);
    VectorXd out_filtered = get_iir();
    shiftX(1);
    shiftY(1);
    addY(out_filtered);

    return out_filtered;
  }

private:
  int      N_elements_;
  int      order_;
  VectorXd a_coef_;
  VectorXd b_coef_;
  MatrixXd Xmat_;
  MatrixXd Ymat_;
  bool     flag_initialized;
}; // class end
}
#endif
