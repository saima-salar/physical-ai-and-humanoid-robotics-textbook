import React, { useState, useEffect, useContext } from 'react';
import { useAuth } from './AuthProvider';

// Context to manage user preferences and personalization data
export const PersonalizationContext = React.createContext();

// Helper function to get chapter Id from URL
const getChapterIdFromURL = () => {
  if (typeof window !== 'undefined') {
    const pathParts = window.location.pathname.split('/');
    return pathParts[pathParts.length - 1] || pathParts[pathParts.length - 2];
  }
  return '';
};

// Provider component to wrap the app
export const PersonalizationProvider = ({ children }) => {
  const { session } = useAuth();
  const [userProfile, setUserProfile] = useState({
    name: '',
    interests: [],
    skillLevel: 'intermediate', // beginner, intermediate, advanced
    learningGoals: [],
    domain: 'general', // robotics, ai, software, etc.
    is_authenticated: false,
    softwareExperience: '',
    hardwareExperience: '',
    programmingLanguages: [],
    roboticsExperience: '',
  });

  const [currentPreferences, setCurrentPreferences] = useState({});
  const [isPersonalizationEnabled, setIsPersonalizationEnabled] = useState(false);

  // Load user profile and preferences on component mount and when session changes
  useEffect(() => {
    loadUserProfile();
    loadChapterPreferences();
  }, [session]);

  const loadUserProfile = () => {
    if (session && session.user) {
      // Load profile from auth session
      const authProfile = {
        name: session.user.name || session.user.email.split('@')[0],
        is_authenticated: true,
        // Map custom fields from Better Auth user object (fallback to localStorage if not available)
        softwareExperience: session.user.softwareExperience || localStorage.getItem('softwareExperience') || '',
        hardwareExperience: session.user.hardwareExperience || localStorage.getItem('hardwareExperience') || '',
        programmingLanguages: session.user.programmingLanguages ?
                             (typeof session.user.programmingLanguages === 'string' ?
                              JSON.parse(session.user.programmingLanguages) :
                              session.user.programmingLanguages) :
                             JSON.parse(localStorage.getItem('programmingLanguages') || '[]'),
        roboticsExperience: session.user.roboticsExperience || localStorage.getItem('roboticsExperience') || '',
        learningGoals: session.user.learningGoals ?
                      (typeof session.user.learningGoals === 'string' ?
                       JSON.parse(session.user.learningGoals) :
                       session.user.learningGoals) :
                       JSON.parse(localStorage.getItem('learningGoals') || '[]'),
        skillLevel: session.user.skillLevel || localStorage.getItem('skillLevel') || 'intermediate',
        domain: session.user.domainInterest || localStorage.getItem('domain') || 'general',
        interests: session.user.interests ?
                  (typeof session.user.interests === 'string' ?
                   JSON.parse(session.user.interests) :
                   session.user.interests) :
                  JSON.parse(localStorage.getItem('interests') || '["robotics", "ai"]'),
      };
      setUserProfile(authProfile);

      // Save to localStorage for consistency
      localStorage.setItem('userProfile', JSON.stringify(authProfile));
    } else {
      // Load from localStorage if not authenticated
      const savedProfile = localStorage.getItem('userProfile');
      if (savedProfile) {
        setUserProfile(JSON.parse(savedProfile));
      } else {
        // Default profile for demo purposes
        const defaultProfile = {
          name: 'User',
          interests: ['robotics', 'ai'],
          skillLevel: 'intermediate',
          learningGoals: ['master robotics', 'learn ai'],
          domain: 'robotics',
          is_authenticated: false,
          softwareExperience: '',
          hardwareExperience: '',
          programmingLanguages: [],
          roboticsExperience: '',
        };
        setUserProfile(defaultProfile);
        localStorage.setItem('userProfile', JSON.stringify(defaultProfile));
      }
    }
  };

  const loadChapterPreferences = () => {
    const chapterId = getChapterIdFromURL();

    if (chapterId) {
      const savedPrefs = localStorage.getItem(`chapterPrefs_${chapterId}`);
      if (savedPrefs) {
        setCurrentPreferences(JSON.parse(savedPrefs));
      }
    }
  };

  const updateUserProfile = (newProfile) => {
    setUserProfile(newProfile);
    localStorage.setItem('userProfile', JSON.stringify(newProfile));

    // Also save individual fields to localStorage for fallback
    localStorage.setItem('softwareExperience', newProfile.softwareExperience || '');
    localStorage.setItem('hardwareExperience', newProfile.hardwareExperience || '');
    localStorage.setItem('programmingLanguages', JSON.stringify(newProfile.programmingLanguages || []));
    localStorage.setItem('roboticsExperience', newProfile.roboticsExperience || '');
    localStorage.setItem('learningGoals', JSON.stringify(newProfile.learningGoals || []));
    localStorage.setItem('skillLevel', newProfile.skillLevel || 'intermediate');
    localStorage.setItem('domain', newProfile.domain || 'general');
    localStorage.setItem('interests', JSON.stringify(newProfile.interests || ['robotics', 'ai']));
  };

  const updateChapterPreferences = (chapterId, preferences) => {
    setCurrentPreferences(preferences);
    localStorage.setItem(`chapterPrefs_${chapterId}`, JSON.stringify(preferences));
  };

  const value = {
    userProfile,
    currentPreferences,
    isPersonalizationEnabled,
    setIsPersonalizationEnabled,
    updateUserProfile,
    updateChapterPreferences
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

// Hook to use the personalization context
export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};

// Component to handle content personalization
export const PersonalizedContent = ({ children, sectionType = 'general', chapterId }) => {
  const { userProfile, currentPreferences, isPersonalizationEnabled } = usePersonalization();

  if (!isPersonalizationEnabled) {
    return <span>{children}</span>;
  }

  // Apply personalization based on user profile and preferences
  const personalizedContent = personalizeContent(children, userProfile, currentPreferences, sectionType);

  return <span dangerouslySetInnerHTML={{ __html: personalizedContent }} />;
};

// Function to personalize content based on user data
const personalizeContent = (content, userProfile, preferences, sectionType) => {
  let personalized = content;

  // Replace placeholder names with user's name
  if (userProfile.name && userProfile.name !== 'User') {
    personalized = personalized.replace(/you/gi, userProfile.name);
    personalized = personalized.replace(/\byour\b/gi, `${userProfile.name}'s`);
    personalized = personalized.replace(/\bthe user\b/gi, userProfile.name);
  }

  // Adjust content based on skill level
  if (userProfile.skillLevel === 'beginner') {
    // Simplify complex concepts for beginners
    personalized = simplifyContentForBeginners(personalized);
  } else if (userProfile.skillLevel === 'advanced') {
    // Add more complex details for advanced users
    personalized = addComplexDetails(personalized);
  }

  // Adjust content based on domain interests
  if (userProfile.domain === 'robotics') {
    personalized = personalizeForDomain(personalized, 'robotics');
  } else if (userProfile.domain === 'ai') {
    personalized = personalizeForDomain(personalized, 'ai');
  } else if (userProfile.domain === 'humanoid') {
    personalized = personalizeForDomain(personalized, 'humanoid');
  }

  // Apply preference-based modifications
  if (preferences && preferences.code === false) {
    // Remove code blocks if user doesn't want them
    personalized = removeCodeBlocks(personalized);
  }

  if (preferences && preferences.math === false) {
    // Simplify mathematical expressions if user doesn't want them
    personalized = simplifyMath(personalized);
  }

  if (preferences && preferences.practical === true) {
    // Add practical examples
    personalized = addPracticalExamples(personalized);
  }

  // Add dynamic examples based on user background
  if (userProfile.programmingLanguages && userProfile.programmingLanguages.includes('Python')) {
    personalized = addProgrammingExamples(personalized, 'Python');
  }

  if (userProfile.hardwareExperience && userProfile.hardwareExperience.toLowerCase().includes('arduino')) {
    personalized = addHardwareExamples(personalized, 'Arduino');
  }

  return personalized;
};

// Helper functions for different personalization strategies
const simplifyContentForBeginners = (content) => {
  // Add simpler explanations and analogies
  let simplified = content;
  // Replace complex terms with simpler alternatives
  simplified = simplified.replace(/\balgorithm\b/gi, 'process');
  simplified = simplified.replace(/\bimplementation\b/gi, 'setup');
  simplified = simplified.replace(/\bframework\b/gi, 'tool');

  // Add guiding text
  simplified = simplified.replace(/\./g, '.$1');
  return simplified;
};

const addComplexDetails = (content) => {
  // Add more technical depth for advanced users
  let detailed = content;
  // Add more technical information
  return detailed;
};

const personalizeForDomain = (content, domain) => {
  if (domain === 'robotics') {
    // Replace generic terms with robotics-specific ones
    let robotic = content;
    robotic = robotic.replace(/\bsystem\b/gi, 'robotic system');
    robotic = robotic.replace(/\bcomponent\b/gi, 'robotic component');
    robotic = robotic.replace(/\bdevice\b/gi, 'robotic device');
    return robotic;
  } else if (domain === 'ai') {
    // Replace generic terms with AI-specific ones
    let ai = content;
    ai = ai.replace(/\bsystem\b/gi, 'AI system');
    ai = ai.replace(/\balgorithm\b/gi, 'AI algorithm');
    ai = ai.replace(/\bmodel\b/gi, 'AI model');
    return ai;
  } else if (domain === 'humanoid') {
    // Replace generic terms with humanoid-specific ones
    let humanoid = content;
    humanoid = humanoid.replace(/\brobot\b/gi, 'humanoid robot');
    humanoid = humanoid.replace(/\bcontrol system\b/gi, 'humanoid control system');
    humanoid = humanoid.replace(/\bmotion control\b/gi, 'humanoid motion control');
    return humanoid;
  }
  return content;
};

const removeCodeBlocks = (content) => {
  // Remove or simplify code examples
  return content;
};

const simplifyMath = (content) => {
  // Simplify mathematical expressions
  return content;
};

const addPracticalExamples = (content) => {
  // Add practical, real-world examples
  return content;
};

const addProgrammingExamples = (content, language) => {
  // Add examples specific to the user's programming background
  let withExamples = content;
  if (content.toLowerCase().includes('algorithm') && !content.toLowerCase().includes('python')) {
    withExamples += ` In ${language}, this would typically be implemented using functions and data structures.`;
  }
  return withExamples;
};

const addHardwareExamples = (content, platform) => {
  // Add examples specific to the user's hardware background
  let withExamples = content;
  if (content.toLowerCase().includes('sensor') && !content.toLowerCase().includes('arduino')) {
    withExamples += ` For ${platform} users, this sensor can be easily interfaced using analog or digital pins.`;
  }
  return withExamples;
};

// Component to wrap around entire chapters for personalization
export const PersonalizedChapter = ({ children, chapterId, chapterTitle }) => {
  const { isPersonalizationEnabled } = usePersonalization();

  return (
    <div className={isPersonalizationEnabled ? 'personalized-chapter' : ''}>
      {children}
    </div>
  );
};