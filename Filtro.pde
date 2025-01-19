/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 * Adapted by Jakson, june, 7, 2019.
 */

public class SimpleKalmanFilter {
  private float _err_measure;
  private float _err_estimate;
  private float _q;
  private float _current_estimate;
  private float _last_estimate;
  private float _kalman_gain;

  SimpleKalmanFilter(float mea_e, float est_e, float q) {
    this._err_measure = mea_e;
    this._err_estimate = est_e;
    this._q = q;
  };
  
  public float updateEstimate(float mea) {
    this._kalman_gain = _err_estimate/(_err_estimate + _err_measure);
    this._current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    this._err_estimate =  (1.0 - _kalman_gain)*_err_estimate + abs(_last_estimate - _current_estimate)*_q;
    this._last_estimate = _current_estimate;
  
    return _current_estimate;
  };
  
  public void setMeasurementError(float mea_e) {
    this._err_measure = mea_e;
  };
  
  public void setEstimateError(float est_e) {
    this._err_estimate=est_e;
  };
  
  public void setProcessNoise(float q) {
    this._q=q;
  };
  
  public float getKalmanGain() {
    return _kalman_gain;
  };
  
  float getEstimateError() {
    return _err_estimate;
  };

}
