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
	num_particles = 500;

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
		weights.push_back(1.0);
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
			particles[i].theta += yaw_rate*delta_t;
		}
		else{ 
			particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
			particles[i].theta += yaw_rate*delta_t;
		}

		// add gaussian noise
		particles[i].x += dist_xpos(gen);
		particles[i].y += dist_ypos(gen);
		particles[i].theta += dist_thetapos(gen);

		// angle_norm(particles[i].theta);
	}

}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	// dataAssociation(in_range_landmarks_, transformed_observations);


	// nearest neighbor data association
	for(int i=0; i<observations.size(); i++){
		double min_dist = 9999999.9;

		// find nearest observation
		for(int j=0; j<predicted.size(); j++){
			double dist_ = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			if(dist_ < min_dist){
				min_dist = dist_;
				observations[i].id = predicted[j].id;
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

	// weights initialization
	weights.clear();

	for(int i=0; i<num_particles; i++){
		vector<LandmarkObs> in_range_landmarks_;
		vector<LandmarkObs> transformed_observations;

		for(int l=0; l<map_landmarks.landmark_list.size(); l++){

			// find landmarks closer than sensor range
			double distance_ = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f);

			if(distance_ < sensor_range){
				LandmarkObs in_range_landmark;
				in_range_landmark.id = map_landmarks.landmark_list[l].id_i;
				in_range_landmark.x = map_landmarks.landmark_list[l].x_f;
				in_range_landmark.y = map_landmarks.landmark_list[l].y_f;

				in_range_landmarks_.push_back(in_range_landmark);
			}

		}

        
		// measurement transformation - vehicle's coordinate to map's coordinate
		for(int j=0; j<observations.size(); j++){
			LandmarkObs transformed_observation_;
			
			transformed_observation_.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
			transformed_observation_.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
			transformed_observation_.id = -1;

			transformed_observations.push_back(transformed_observation_);
		}


		// data assosication
		dataAssociation(in_range_landmarks_, transformed_observations);

		// calculate multivariate gaussian probability
		double mult_gaussian_prob;
		double prob_normalizer;
		double weight_ = 0.0;
		double land_x_veh, land_y_veh;
		double x_diff, y_diff, x_diff2, y_diff2;


		if(observations.size() > 0){
			weight_ = 1.0;
			for(int l=0; l<observations.size(); l++){
				x_diff = 100.0;
				y_diff = 100.0;

				for(int k=0; k<in_range_landmarks_.size(); k++){
					if(transformed_observations[l].id == in_range_landmarks_[k].id){
						// predicted landmark location in vehicle's coordinate
						// land_x_veh = (in_range_landmarks_[k].x - particles[i].x) * cos(particles[i].theta) - (in_range_landmarks_[k].y - particles[i].y) * sin(particles[i].theta);
						// land_y_veh = (in_range_landmarks_[k].x - particles[i].x) * sin(particles[i].theta) + (in_range_landmarks_[k].y - particles[i].y) * cos(particles[i].theta);
						
						// x_diff  = observations[l].x - land_x_veh;
						// y_diff  = observations[l].y - land_y_veh;
						
						x_diff = transformed_observations[l].x - in_range_landmarks_[k].x;
						y_diff = transformed_observations[l].y - in_range_landmarks_[k].y;
						break;
						// cout << "x_diff: " << x_diff << endl;
						// cout << "y_diff: " << y_diff << endl;
					}
				}

				x_diff2 = x_diff * x_diff;
				y_diff2 = y_diff * y_diff;

				// cout << "x_diff2: " << x_diff2 << endl;
				// cout << "y_diff2: " << y_diff2 << endl;

				prob_normalizer = 1 / (sqrt(2 * M_PI) * std_obsx * std_obsy);
				mult_gaussian_prob = prob_normalizer * exp(-1.0/2.0 * (x_diff2/(std_obsx*std_obsx) + y_diff2/(std_obsy*std_obsy)));
				// cout << "prob_normalizer: " << prob_normalizer << endl;
				// cout << "mult_gaussian_prob: " << mult_gaussian_prob << endl;
				// cout << "x_diff2/(std_obsx*std_obsx) + y_diff2/(std_obsy*std_obsy): " << x_diff2/(std_obsx*std_obsx) + y_diff2/(std_obsy*std_obsy) << endl;
				weight_ *= mult_gaussian_prob;
			}
		}
		
		// cout << weight_ << endl;
		particles[i].weight = weight_;
		weights.push_back(weight_);
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	// set discrete distribution of particles by weights
	default_random_engine gen;
	discrete_distribution<int> particle_dist {weights.begin(), weights.end()};

	vector<Particle> particles_resampled;
	particles_resampled.resize(num_particles);

	for(int i=0; i<num_particles; i++){
		particles_resampled[i] = particles[particle_dist(gen)];
	}

	particles = particles_resampled;

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
