  #pragma once

namespace local_grid_map {

/*!
 * Structure containing the parameters of the camera.
 */
typedef struct
{
	int height;
	int width;
	int fps;
} CamSettings;

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Set the camera settings parameters.
   * @param camSettings wraps the settings of the camera.
   */
  void setCamSettings(const CamSettings cam_settings);

  /*!
   * Get the camera settings parameters.
   * @return Settings of the camera.
   */
  CamSettings getCamSettings();

  /*!
   * Set the camera configuration request.
   * @param conf_camera set true for a new configuration.
   */
  void setCamIsToConf(const bool is_toconf);

  /*!
   * Check if the camera should be configured.
   * @return if the camera should be configured.
   */
  bool getCamIsToConf();

  /*!
   * Start or stop the camera.
   * @param run_camera set true to run the camera.
   */
  void setCamIsRunning(const bool is_running);

  /*!
   * Check if the camera should run.
   * @return if the camera should run.
   */
  bool getCamIsRunning();

  /*!
   * Set stereo mode.
   * @param run_camera set true to run the camera.
   */
  void setCamIsStereo(const bool is_stereo);

  /*!
   * Check if the camera is in stereo mode.
   * @return if the camera is in stereo mode.
   */
  bool getCamIsStereo();

 private:

  //! Internal logical variable to configure the camera only on service request.
  bool istoconf_;

  //! Internal logical variable to run the camera.
  bool isrunning_;

  //! Internal camera settings wrapper
  CamSettings camsettings_;

};

} /* namespace */
