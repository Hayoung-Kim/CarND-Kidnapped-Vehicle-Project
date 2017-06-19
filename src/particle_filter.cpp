/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	// set the number of particles
	num_particles = 200;

	// set standard deviations for x, y, theta
	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// create normal distribution for x, y, theta
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// set single particle 
	Particle single_particle;

	// initialize particles with gaussian distribution
	for(int i=0; i<num_particles; i++){
		single_particle.id = i;
		single_particle.x = dist_x(gen);
		single_particle.y = dist_y(gen);
		single_particle.theta = dist_theta(gen);
		single_particle.weight = 1.0;
		particles.push_back(single_particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// set normal distribution for x,y,theta
	default_random_engine gen;
	normal_distribution<double> dist_xpos(0, std_pos[0]);
	normal_distribution<double> dist_ypos(0, std_pos[1]);
	normal_distribution<double> dist_thetapos(0, std_pos[2]);
	
	// predict particles
	for(int i=0; i<num_particles; i++){
		// nominal value prediction
		if(fabs(yaw_rate)<0.001){
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else{ 
			particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			particles[i].theta += yaw_rate*delta_t;
		}

		// add gaussian noise
		particles[i].x += dist_xpos(gen);
		particles[i].y += dist_ypos(gen);
		particles[i].theta += dist_thetapos(gen);
	}

}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	vector<LandmarkObs> temp;

	// nearest neighbor data association
	for(int i=0; i<predicted.size(); i++){
		double x_pred = predicted[i].x;
		double y_pred = predicted[i].y;
		double min_dist = 100000;

		// find nearest observation
		for(j=0; j<observation.size(); j++){
			double dx = x_pred - observation[j].x;
			double dy = y_pred - observation[j].y;

			double dist_ = sqrt(dx*dx + dy*dy);

			if(dist_ < min_dist){
				min_dist = dist_;
				double matched_x = observation[j].x;
				double matched_y = observation[j].y;

			}

		}
	}


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	// parse std_landmark[]
	double std_obsx = std_landmark[0];
	double std_obsy = std_landmark[1];

	vector<LandmarkObs> in_range_landmarks_;
	vector<LandmarkObs> predicted;

	for(int i=0; i<num_particles; i++){
		

		for(int l=0; l<map_landmarks.size(); l++){

			// find landmarks closer than sensor range
			double dx_ = particles[i].x - map_landmarks[l].x_f;
			double dy_ = particles[i].y - map_landmarks[l].y_f;
			double distance_ = sqrt(dx_*dx_ + dy_*dy_);

			if(distance_ <= sensor_range){
				LandmarkObs in_range_landmark;
				in_range_landmark.id = map_landmarks[l].id_i;
				in_range_landmark.x = map_landmarks[l].x_f;
				in_range_landmark.y = map_landmarks[l].y_f;

				in_range_landmarks_.push_back(in_range_landmark);
			}

		}

		// predicted observation (global to vehicle coordinate)
		for(int k=0; k<in_range_landmarks_.size(); k++){
			LandmarkObs predicted_;

			predicted_.id = in_range_landmarks_[k].id;
			
			double X_, Y_;
			X_ = in_range_landmarks_[k].x_f - particles[i].x;
			Y_ = in_range_landmarks_[k].y_f - particles[i].y;
			
			double cc, ss;
			cc = cos(particles[i].theta);
			ss = sin(particles[i].theta);

			predicted_.x = X_ + X_ * cc - Y_ * ss;
			predicted_.y = Y_ + X_ * ss + Y_ * cc;
			
			predicted.push_back(predicted_);
		}

		// data assosication
		dataAssociation(predicted, observations)

		// calculate multivariate gaussian probability
		double mult_gaussian_prob;
		double weight_ = 1.0;
		for(int l=0; l<observations.size(); l++){
			double e_x = predicted[l].x - observation[l].x;
			double e_y = predicted[l].y - observation[l].y;
			double prob_normalizer = 1/(sqrt(2 * M_PI) * std_obsx * std_obsy);
			mult_gaussian_prob = prob_normalizer * exp(-1/2*(e_x*e_x/(std_obsx*std_obsx) + e_y*e_y/(std_obsy*std_obsy));

			weight_ *= mult_gaussian_prob;
		}

		particles[i].weight = weight_;
		weights.push_back(weight_);
		
		// clear vector to update next particle.
		in_range_landmarks_.clear();
		predicted.clear();
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
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
