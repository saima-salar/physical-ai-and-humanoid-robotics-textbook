import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/AuthProvider';
import { usePersonalization } from '../components/PersonalizationProvider';
import '../components/AuthForm.css';
import '../components/ProfilePage.css';

const ProfilePage = () => {
  const { session, isLoading } = useAuth();
  const { userProfile, updateUserProfile } = usePersonalization();
  const [formData, setFormData] = useState({
    softwareExperience: '',
    hardwareExperience: '',
    programmingLanguages: [],
    roboticsExperience: '',
    learningGoals: [],
    skillLevel: '',
    domainInterest: '',
  });
  const [isEditing, setIsEditing] = useState(false);
  const [message, setMessage] = useState('');

  useEffect(() => {
    if (userProfile) {
      setFormData({
        softwareExperience: userProfile.softwareExperience || '',
        hardwareExperience: userProfile.hardwareExperience || '',
        programmingLanguages: userProfile.programmingLanguages || [],
        roboticsExperience: userProfile.roboticsExperience || '',
        learningGoals: userProfile.learningGoals || [],
        skillLevel: userProfile.skillLevel || '',
        domainInterest: userProfile.domain || '',
      });
    }
  }, [userProfile]);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value
    });
  };

  const handleMultiSelectChange = (option, field) => {
    const currentValues = formData[field] || [];
    if (currentValues.includes(option)) {
      setFormData({
        ...formData,
        [field]: currentValues.filter(item => item !== option)
      });
    } else {
      setFormData({
        ...formData,
        [field]: [...currentValues, option]
      });
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      // Update user profile
      const updatedProfile = {
        ...userProfile,
        ...formData,
        programmingLanguages: formData.programmingLanguages,
        learningGoals: formData.learningGoals,
        skillLevel: formData.skillLevel,
        domain: formData.domainInterest,
      };

      updateUserProfile(updatedProfile);
      setMessage('Profile updated successfully!');
      setIsEditing(false);

      // In a real implementation, you would also update the server-side profile
      // await updateServerProfile(formData);
    } catch (error) {
      setMessage('Error updating profile. Please try again.');
      console.error('Profile update error:', error);
    }
  };

  if (isLoading) {
    return (
      <Layout title="Loading Profile...">
        <div className="auth-container">
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  if (!session) {
    return (
      <Layout title="Profile" description="Sign in to view your profile">
        <div className="auth-container">
          <p>Please <a href="/signin">sign in</a> to view your profile.</p>
        </div>
      </Layout>
    );
  }

  // Options for multi-select fields
  const programmingLanguageOptions = [
    'Python', 'C++', 'C', 'Java', 'JavaScript', 'TypeScript', 'ROS', 'MATLAB', 'Rust', 'Other'
  ];

  const learningGoalOptions = [
    'Learn robotics fundamentals',
    'Build humanoid robots',
    'Understand AI in robotics',
    'Develop control systems',
    'Work on computer vision',
    'Explore machine learning',
    'Other'
  ];

  return (
    <Layout title="User Profile" description="Manage your profile and background information">
      <div className="auth-container">
        <h2>User Profile</h2>
        <p>Welcome, {session.user?.name || session.user?.email.split('@')[0]}!</p>

        {message && (
          <div className={`message ${message.includes('successfully') ? 'success' : 'error'}`}>
            {message}
          </div>
        )}

        {isEditing ? (
          <form onSubmit={handleSubmit} className="profile-form">
            <div className="form-section">
              <h3>Software Background</h3>

              <div className="form-group">
                <label htmlFor="softwareExperience">Software Experience:</label>
                <textarea
                  id="softwareExperience"
                  name="softwareExperience"
                  value={formData.softwareExperience}
                  onChange={handleInputChange}
                  placeholder="e.g., I have 3 years of experience with Python, worked on embedded systems, etc."
                />
              </div>

              <div className="form-group">
                <label>Programming languages you're familiar with:</label>
                <div className="multi-select-options">
                  {programmingLanguageOptions.map(option => (
                    <label key={option} className="checkbox-option">
                      <input
                        type="checkbox"
                        checked={formData.programmingLanguages.includes(option)}
                        onChange={() => handleMultiSelectChange(option, 'programmingLanguages')}
                      />
                      <span>{option}</span>
                    </label>
                  ))}
                </div>
              </div>
            </div>

            <div className="form-section">
              <h3>Hardware Background</h3>

              <div className="form-group">
                <label htmlFor="hardwareExperience">Hardware/Robotics Experience:</label>
                <textarea
                  id="hardwareExperience"
                  name="hardwareExperience"
                  value={formData.hardwareExperience}
                  onChange={handleInputChange}
                  placeholder="e.g., I've worked with Arduino, Raspberry Pi, motor controllers, etc."
                />
              </div>

              <div className="form-group">
                <label htmlFor="roboticsExperience">Robotics Experience:</label>
                <textarea
                  id="roboticsExperience"
                  name="roboticsExperience"
                  value={formData.roboticsExperience}
                  onChange={handleInputChange}
                  placeholder="e.g., I've built mobile robots, worked on manipulators, etc."
                />
              </div>
            </div>

            <div className="form-section">
              <h3>Learning Goals</h3>

              <div className="form-group">
                <label>Learning goals:</label>
                <div className="multi-select-options">
                  {learningGoalOptions.map(option => (
                    <label key={option} className="checkbox-option">
                      <input
                        type="checkbox"
                        checked={formData.learningGoals.includes(option)}
                        onChange={() => handleMultiSelectChange(option, 'learningGoals')}
                      />
                      <span>{option}</span>
                    </label>
                  ))}
                </div>
              </div>

              <div className="form-group">
                <label htmlFor="skillLevel">Skill Level:</label>
                <select
                  id="skillLevel"
                  name="skillLevel"
                  value={formData.skillLevel}
                  onChange={handleInputChange}
                >
                  <option value="">Select your skill level</option>
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="domainInterest">Area of Interest:</label>
                <select
                  id="domainInterest"
                  name="domainInterest"
                  value={formData.domainInterest}
                  onChange={handleInputChange}
                >
                  <option value="">Select an area</option>
                  <option value="robotics">Robotics</option>
                  <option value="ai">Artificial Intelligence</option>
                  <option value="humanoid">Humanoid Robotics</option>
                  <option value="control">Control Systems</option>
                  <option value="sensors">Sensors & Perception</option>
                  <option value="software">Software Development</option>
                </select>
              </div>
            </div>

            <div className="form-actions">
              <button type="submit" className="submit-btn">
                Save Changes
              </button>
              <button
                type="button"
                className="cancel-btn"
                onClick={() => {
                  setIsEditing(false);
                  // Reset form to original values
                  if (userProfile) {
                    setFormData({
                      softwareExperience: userProfile.softwareExperience || '',
                      hardwareExperience: userProfile.hardwareExperience || '',
                      programmingLanguages: userProfile.programmingLanguages || [],
                      roboticsExperience: userProfile.roboticsExperience || '',
                      learningGoals: userProfile.learningGoals || [],
                      skillLevel: userProfile.skillLevel || '',
                      domainInterest: userProfile.domain || '',
                    });
                  }
                }}
              >
                Cancel
              </button>
            </div>
          </form>
        ) : (
          <div className="profile-view">
            <div className="profile-info">
              <h3>Account Information</h3>
              <p><strong>Email:</strong> {session.user?.email}</p>
              {session.user?.name && <p><strong>Name:</strong> {session.user.name}</p>}
            </div>

            <div className="profile-info">
              <h3>Software Background</h3>
              <p><strong>Experience:</strong> {userProfile.softwareExperience || 'Not provided'}</p>
              <p><strong>Languages:</strong> {userProfile.programmingLanguages?.length > 0 ? userProfile.programmingLanguages.join(', ') : 'Not provided'}</p>
            </div>

            <div className="profile-info">
              <h3>Hardware Background</h3>
              <p><strong>Experience:</strong> {userProfile.hardwareExperience || 'Not provided'}</p>
              <p><strong>Robotics:</strong> {userProfile.roboticsExperience || 'Not provided'}</p>
            </div>

            <div className="profile-info">
              <h3>Learning Goals</h3>
              <p><strong>Goals:</strong> {userProfile.learningGoals?.length > 0 ? userProfile.learningGoals.join(', ') : 'Not provided'}</p>
              <p><strong>Skill Level:</strong> {userProfile.skillLevel || 'Not provided'}</p>
              <p><strong>Interest:</strong> {userProfile.domain || 'Not provided'}</p>
            </div>

            <button
              className="edit-btn"
              onClick={() => setIsEditing(true)}
            >
              Edit Profile
            </button>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default ProfilePage;