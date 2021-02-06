/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::normal_distribution;
using std::string;
using std::vector;

#define EPS 0.00001

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 100; // TODO: Set the number of particles

  // Check if ParticleFilter is already initialized
  if (is_initialized) {
    return;
  }

  // Standard deviation values for x, y and theta
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Create normal distributions using std::normal_distribution
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // Initialize the particles based on GPS readings with a random Gaussian
  // noise, and each particle has initially a weight of 1.
  for (int i = 0; i < num_particles; ++i) {
    Particle particle;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    particles.push_back(particle);
  }

  // Initialization process is done
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   *
   */
  // Standard deviation values for x, y and theta
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  // Create normal distributions using std::normal_distribution
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // Loop on each particle
  for (int i = 0; i < num_particles; i++) {
    double theta = particles[i].theta;

    // Calculate new state.

    // If yaw equals to zero
    if (fabs(yaw_rate) < EPS) {
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
    }
    // The equations for updating x, y and the yaw angle when the yaw rate is
    // not equal to zero:
    else {
      particles[i].x +=
          velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      particles[i].y +=
          velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Adding noise.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */
  unsigned int nObservations = observations.size();
  unsigned int nPredictions = predicted.size();

  for (unsigned int i = 0; i < nObservations; i++) {
    // Initialize min distance with maximum possible number.
    double minDistance = std::numeric_limits<double>::max();

    // Initialize id of landmark from map to be associated with the observation.
    int mapId = -1;

    for (unsigned j = 0; j < nPredictions; j++) {
      // Calculate distance between current observation and prediction
      double distance = dist(observations[i].x, observations[i].y,
                             predicted[j].x, predicted[j].y);

      // If the "distance" is less than min, stored the id and update min.
      if (distance < minDistance) {
        minDistance = distance;
        mapId = predicted[j].id;
      }
    }

    // Update the observation identifier to the closest predicted landmark's id.
    observations[i].id = mapId;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no
   * scaling). The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  // Standard deviation values for x, y for landmarks
  double stdDevLandmarkX = std_landmark[0];
  double stdDevLandmarkY = std_landmark[1];

  // Loop on all particles
  for (int i = 0; i < num_particles; i++) {
    double particleX = particles[i].x;
    double particleY = particles[i].y;
    double particleTheta = particles[i].theta;

    // Vector of map landmark locations predictions within sensor range
    vector<LandmarkObs> close_landmarks;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // x,y,id of the landmark
      float landmarkX = map_landmarks.landmark_list[j].x_f;
      float landmarkY = map_landmarks.landmark_list[j].y_f;
      int id = map_landmarks.landmark_list[j].id_i;
      // Calculate distance between landmark observation and particle
      double landmarkDist = dist(particleX, particleY, landmarkX, landmarkY);

      if (landmarkDist <= sensor_range) {
        close_landmarks.push_back(LandmarkObs{id, landmarkX, landmarkY});
      }
    }

    // Transform observations from vehicle's coordinate system to map coordinate
    // system
    vector<LandmarkObs> transformedObservations;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double observationsX = observations[j].x;
      double observationsY = observations[j].y;
      int observationsId = observations[j].id;

      double transformedX = cos(particleTheta) * observationsX -
                            sin(particleTheta) * observationsY + particleX;
      double transformedY = sin(particleTheta) * observationsX +
                            cos(particleTheta) * observationsY + particleY;
      transformedObservations.push_back(
          LandmarkObs{observationsId, transformedX, transformedY});
    }

    // Association of observations to close landmarks
    dataAssociation(close_landmarks, transformedObservations);

    // Reinitialize particle weight.
    particles[i].weight = 1.0;

    // Calculate weights.
    for (unsigned int j = 0; j < transformedObservations.size(); j++) {
      // x,y,id of closest landmark
      double transformedObservationX = transformedObservations[j].x;
      double transformedObservationY = transformedObservations[j].y;
      int landmarkId = transformedObservations[j].id;

      double associatedLandmarkX, associatedLandmarkY;
      unsigned int k = 0;
      unsigned int nLandmarks = close_landmarks.size();
      bool found = false;

      // get x,y of the landmark associated with the current observation id
      while (!found && k < nLandmarks) {
        if (close_landmarks[k].id == landmarkId) {
          found = true;
          associatedLandmarkX = close_landmarks[k].x;
          associatedLandmarkY = close_landmarks[k].y;
        }
        k++;
      }

      // Calculating weight using multivariate Gaussian
      double dX = transformedObservationX - associatedLandmarkX;
      double dY = transformedObservationY - associatedLandmarkY;

      // Multivariate-Gaussian Probability
      double weight =
          (1 / (2 * M_PI * stdDevLandmarkX * stdDevLandmarkY)) *
          exp(-(dX * dX / (2 * stdDevLandmarkX * stdDevLandmarkX) +
                (dY * dY / (2 * stdDevLandmarkY * stdDevLandmarkY))));
      if (weight == 0) {
        // avoid weight of zero
        particles[i].weight *= EPS;
      } else {
        particles[i].weight *= weight;
      }
    }
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // Get all weights.
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // get max weight.
  double max_weight = *max_element(weights.begin(), weights.end());

  // Random starting index for resampling wheel.
  std::uniform_int_distribution<int> distInt(0, num_particles - 1);
  int index = distInt(gen);

  double beta = 0.0;

  // Uniform random distribution [0.0, max_weight).
  std::uniform_real_distribution<double> weightsDist(0.0, max_weight);

  // Resampling wheel.
  vector<Particle> resampledParticles;
  for (int i = 0; i < num_particles; i++) {
    beta += weightsDist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }

  particles = resampledParticles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  // Clear the previous associations
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}