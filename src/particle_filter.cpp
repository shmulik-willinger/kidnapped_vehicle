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

static default_random_engine gen;

// Set the number of particles. 
// Creates a normal (Gaussian) distribution for x, y and theta (yaw).
// Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1. 
// Add random Gaussian noise to each particle.
void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
	num_particles = 100;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	particles.resize(num_particles);
	for (int i = 0; i < num_particles; i++)
	{
		particles[i].id = i;
		particles[i].x = x + dist_x(gen);
		particles[i].y = y + dist_y(gen);
		particles[i].theta = theta + dist_theta(gen);
		particles[i].weight = 1.0;
	}
	is_initialized = true;
}

// Add measurements and random Gaussian noise to each particle 
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
{
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; i++)
	{
		if (abs(yaw_rate) != 0)
		{
			particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
			particles[i].theta += yaw_rate * delta_t;
		}
		else
		{
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}

		// Add noise to the particles
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

// Find the predicted measurement that is closest to each observed measurement and assign the 
//   observed measurement to this particular landmark.
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) 
{



}

// Update the weights of each particle using a mult-variate Gaussian distribution.
// The observations are given in the VEHICLE'S coordinate system. Your particles are located
//   according to the MAP'S coordinate system. You will need to transform between the two systems.
//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
//   The following is a good resource for the theory:  https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
//   and the following is a good resource for the actual equation to implement (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations, const Map &map_landmarks) 
{
	for (int i = 0; i < num_particles; i++)
	{
		// this vector collects all the map landmark locations that are in the sensor range
		vector<LandmarkObs> predictions;
		int landmarks_size = map_landmarks.landmark_list.size();

		for (int j = 0; j < landmarks_size; j++)
		{
			float x_range = map_landmarks.landmark_list[j].x_f - particles[i].x;
			float y_range = map_landmarks.landmark_list[j].y_f - particles[i].y;
			// only the landmarks within the sensor range of the particle will be added to the vector
			if (fabs(x_range) <= sensor_range && fabs(y_range) <= sensor_range)
				predictions.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });
		}

		// transform between the VEHICLE'S coordinate system to the MAP'S coordinate system
		vector<LandmarkObs> transformed;
		int observations_size = observations.size();

		for (int j = 0; j < observations_size; j++)
		{
			float p_theta = particles[i].theta;
			float x_transform = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + particles[i].x;
			float y_transform = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + particles[i].y;
			transformed.push_back(LandmarkObs{ observations[j].id, x_transform, y_transform });
		}

		// Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
		dataAssociation(predictions, transformed);

		int transform_size = transformed.size();
		int predictions_size = predictions.size();

		for (int j = 0; j < transform_size; j++)
		{
			for (int k = 0; k < predictions_size; k++)
			{
				if (predictions[k].id == transformed[j].id)
				{
					float x_transform = transformed[j].x;
					float y_transform = transformed[j].y;

					float x_prediction = predictions[k].x;
					float y_prediction = predictions[k].y;

					float x_landmark = std_landmark[0];
					float y_landmark = std_landmark[1];

					// weight for the observation with multivariate Gaussian
					particles[i].weight *= (1 / (2 * M_PI * x_landmark * y_landmark)) * exp(-(pow(x_prediction - x_transform, 2) / (2 * pow(x_landmark, 2)) + (pow(y_prediction - y_transform, 2) / (2 * pow(y_landmark, 2)))));
				}
			}
		}
	}
}

void ParticleFilter::resample() 
{
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
