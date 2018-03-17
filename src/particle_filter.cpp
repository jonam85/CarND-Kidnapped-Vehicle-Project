/*
 * particle_filter.cpp
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
  // Setting the total number of Particles used for the filter
  num_particles = 100;
  
  // Using the default_random_engine for adding noise
  default_random_engine gen;
    
  // These lines create a normal (Gaussian) distribution for x, y and theta.  
  normal_distribution<double> dist_x(x, std[0]);   
  normal_distribution<double> dist_y(y, std[1]); 
  normal_distribution<double> dist_theta(theta, std[2]); 
    
  // Initialize all the particles with Weight as 1 and random normal distribution based on GPS
  for (int i = 0; i < num_particles; ++i) 
  {   
    Particle p;
    p.id = i;
    
    p.x = dist_x(gen);   
    p.y = dist_y(gen);   
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    
    // Insert the particle to the particles vector
    particles.push_back(p);
    weights.push_back(1.0);
  }
  
  // Set the initialized flag to true
  is_initialized = true;
  
  //cout << "Debug: Particles Initialized" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  default_random_engine gen;
  
  for (int i = 0; i < num_particles; ++i) 
  {   
    double new_x, new_y, new_theta;
    
    // Using Velocity and Yawrate to predict the next states for position and orientation
    
    // If yaw_rate is zero, use the formula for straight line
    if(fabs(yaw_rate) < 0.0001) 
    {
      new_x = particles[i].x + (velocity * delta_t * cos(particles[i].theta));
      new_y = particles[i].y + (velocity * delta_t * sin(particles[i].theta));
      new_theta = particles[i].theta;
    } 
    else  // Yaw rate is non-zero, using formula for curve
    { 
      new_x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
      new_y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
      new_theta = particles[i].theta + yaw_rate * delta_t;
    }
    
    // Adding Normal distribution noise to the predicted positions
    normal_distribution<double> dist_x(new_x, std_pos[0]);   
    normal_distribution<double> dist_y(new_y, std_pos[1]); 
    normal_distribution<double> dist_theta(new_theta, std_pos[2]); 
  
    // Updating the Predicted positions
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    
  }
  
  //cout << "Debug: Particles Predicted" << endl;

}

// Note: This function is not called. Only for understanding purpose this function is used.
  
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  
  for (int i = 0; i < observations.size(); i++) {

    int nearest_j = -1;
    double best_error = 10000;

    for (int j = 0; j < predicted.size(); j++) {
      
      double dx = predicted[j].x - observations[i].x;
      double dy = predicted[j].y - observations[i].y;
      double error = sqrt((dx*dx) + (dy*dy));

      if (error < best_error) {
        nearest_j = predicted[j].id;
        best_error = error;
      }
    }
    observations[i].id = nearest_j;
  }
  

  //  cout << "Debug: Particles Associated" << endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    // constants used later for calculating the new weights


  // loop through all particles
  for (int  i = 0; i < num_particles; i++) 
  {

    // Gather all Map data and use only the nearest landmarks for each particle
    vector<LandmarkObs> landmarks_in_range;

    // Loop through all the Landmarks
    for (int j = 0;  j < map_landmarks.landmark_list.size(); j++) 
    {

      int mid = map_landmarks.landmark_list[j].id_i;
      double mx = map_landmarks.landmark_list[j].x_f;
      double my = map_landmarks.landmark_list[j].y_f;
      
      // Find the distance beween particle and landmarks 
      double dx = mx - particles[i].x;
      double dy = my - particles[i].y;
      double error = sqrt(dx * dx + dy * dy);
      
      // and use only the landmarks in sensor range
      if (error < sensor_range) {

        LandmarkObs landmark_in_range = 
        {
          mid,
          mx,
          my
        };
         
        // Using vector to store the landmarks in range
        landmarks_in_range.push_back(landmark_in_range);
      }
    }
    
    // Getting sensor observations and associating with the landmarks in range
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    
    vector<LandmarkObs> map_observations;
    
    // loop through all observations
    for (int j = 0; j < observations.size(); j++)
    {

      LandmarkObs obs;
      
      // Convert vehicle co-ordinates to map co-ordinates
      obs.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
      obs.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
      
      // Using the nearest neighbour to associate the sensor observations to landmarks in range
      int nearest = -1;
      double best_error = 10000;
  
      for (int k = 0; k < landmarks_in_range.size(); k++) {
        
        // Finding the distance between observations and landmarks
        double dx = landmarks_in_range[k].x - obs.x;
        double dy = landmarks_in_range[k].y - obs.y;
        double error = sqrt((dx*dx) + (dy*dy));
  
        if (error < best_error) {
          nearest = landmarks_in_range[k].id;
          best_error = error;
        }
      }
      obs.id = nearest;
      
      // Updating the associations vectors and map_observations vectors for further use
      
      associations.push_back(obs.id);
      sense_x.push_back(obs.x);
      sense_y.push_back(obs.y);
      
      map_observations.push_back(obs);

    }


    //Association for visualization
    SetAssociations(particles[i], associations, sense_x, sense_y);

    
    //Update Particle Weights to 1 for finding probability
    double w = 1.0;
    
    
    // Loop through all map observations for x and y 
    for (int j = 0; j < map_observations.size(); j++)
    {
      double dx = map_observations[j].x;
      double dy = map_observations[j].y;
      
      // Loop through the landmarks in range for mu_x and mu_y
      for (int k = 0; k < landmarks_in_range.size(); k++)
      {
        if(map_observations[j].id == landmarks_in_range[k].id)
        {
          dx -= (landmarks_in_range[k].x);
          dy -= (landmarks_in_range[k].y);
          break;
        }
      }
      

      // Check for divide by zero
      if(fabs(dx) < 0.0001)
        dx = 0.0001;
      if(fabs(dy) < 0.0001)
        dy = 0.0001;

      // Finding Multivariate-Gaussian Probability
      double a = (0.5 / (std_landmark[0] * std_landmark[0])) * dx * dx;
      double b = (0.5 / (std_landmark[1] * std_landmark[1])) * dy * dy;
      double prob = exp(-(a + b)) / (( 2.0 * M_PI * std_landmark[0] * std_landmark[1]));
      
      // Updating the weight by multiplying all probabilities together
      w *= prob;
    }

    // Update weights for associated particle
    particles[i].weight = w;
    weights[i] = w;
  }
  
  //cout << "Debug: Particles Weights updated" << endl;
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  vector<Particle> sample_particles (num_particles);
  
  // Initializes discrete distribution function
  random_device rd;
  mt19937 gen(rd());
  discrete_distribution<int> disc_distribution(weights.begin(), weights.end());

  // Loop through all particles and resample to merge them based on Weights distribution
  for (int i = 0; i < num_particles; i++) {
   
    sample_particles[i] = particles[disc_distribution(gen)];

  }
  particles = sample_particles;
   
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    
      //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
  
  return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
