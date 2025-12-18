import React, { useState, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';

// Function to transform content based on user profile and preferences
export const transformContent = (content, userProfile, preferences, chapterContext = {}) => {
  // Convert content to string if it's not already
  let transformedContent = typeof content === 'string' ? content : String(content);

  // 1. Personalize based on user name
  if (userProfile.name && userProfile.name !== '' && userProfile.name !== 'User') {
    transformedContent = transformedContent
      .replace(/\byou\b/gi, userProfile.name)
      .replace(/\byour\b/gi, `${userProfile.name}'s`)
      .replace(/\buser\b(?!\s*authentication)/gi, userProfile.name);
  }

  // 2. Adjust complexity based on skill level
  transformedContent = adjustComplexity(transformedContent, userProfile.skillLevel, preferences);

  // 3. Add domain-specific examples
  transformedContent = addDomainSpecificContent(transformedContent, userProfile.domain, chapterContext);

  // 4. Apply preference filters
  transformedContent = applyPreferenceFilters(transformedContent, preferences);

  // 5. Add personalized examples based on interests
  transformedContent = addPersonalizedExamples(transformedContent, userProfile.interests, chapterContext);

  return transformedContent;
};

const adjustComplexity = (content, skillLevel, preferences) => {
  let adjusted = content;

  if (skillLevel === 'beginner') {
    // Simplify complex concepts for beginners
    adjusted = adjusted
      .replace(/\balgorithm\b/gi, 'step-by-step process')
      .replace(/\bframework\b/gi, 'tool or system')
      .replace(/\bimplementation\b/gi, 'setup or application')
      .replace(/In advanced applications,/gi, 'In more complex applications,')
      .replace(/Advanced techniques include/gi, 'More complex approaches include');

    // Add more explanatory phrases
    adjusted = addBeginnerExplanations(adjusted);
  } else if (skillLevel === 'advanced') {
    // Add more technical depth
    adjusted = addTechnicalDepth(adjusted);
  }

  // If user doesn't want detailed math, simplify
  if (preferences && preferences.math === false) {
    // Replace complex math expressions with simpler explanations
    adjusted = adjusted.replace(/\bmathematical formula|equation|calculation\b/gi, 'calculation method');
  }

  return adjusted;
};

const addBeginnerExplanations = (content) => {
  // Add analogies and simpler explanations
  let withExplanations = content;

  // Add simple analogies for complex concepts
  withExplanations = withExplanations
    .replace(/\bROS 2\b/gi, 'ROS 2 (Robot Operating System 2 - think of it as the robot\'s communication system)')
    .replace(/\bURDF\b/gi, 'URDF (Unified Robot Description Format - like a blueprint for robot structure)')
    .replace(/\bGazebo\b/gi, 'Gazebo (a simulation environment - like a virtual testing ground for robots)')
    .replace(/\bIsaac Sim\b/gi, 'Isaac Sim (NVIDIA\'s simulation platform - a high-quality virtual environment for robots)');

  return withExplanations;
};

const addTechnicalDepth = (content) => {
  // Add more technical details for advanced users
  let withDepth = content;

  // Add technical specifications and deeper explanations
  withDepth = withDepth
    .replace(/The system uses/gi, 'The system implements advanced techniques using')
    .replace(/This works by/gi, 'This works by leveraging sophisticated algorithms and');

  return withDepth;
};

const addDomainSpecificContent = (content, domain, chapterContext) => {
  if (!domain) return content;

  let domainSpecific = content;

  if (domain === 'robotics') {
    // Add robotics-specific context
    domainSpecific = domainSpecific
      .replace(/\bsystem\b/gi, 'robotic system')
      .replace(/\bcomponent\b/gi, 'robotic component')
      .replace(/\bdevice\b/gi, 'robotic device')
      .replace(/\bcontrol\b/gi, 'robotic control')
      .replace(/\bnavigation\b/gi, 'robotic navigation');
  } else if (domain === 'ai') {
    // Add AI-specific context
    domainSpecific = domainSpecific
      .replace(/\bsystem\b/gi, 'AI system')
      .replace(/\balgorithm\b/gi, 'AI algorithm')
      .replace(/\bmodel\b/gi, 'AI model')
      .replace(/\blearning\b/gi, 'machine learning')
      .replace(/\bintelligence\b/gi, 'artificial intelligence');
  } else if (domain === 'software') {
    // Add software-specific context
    domainSpecific = domainSpecific
      .replace(/\bsystem\b/gi, 'software system')
      .replace(/\bcomponent\b/gi, 'software component')
      .replace(/\bprocess\b/gi, 'software process');
  }

  return domainSpecific;
};

const applyPreferenceFilters = (content, preferences) => {
  let filtered = content;

  if (preferences) {
    // Remove/modify content based on user preferences
    if (preferences.code === false) {
      // Remove or hide code blocks
      filtered = filtered.replace(/```[\s\S]*?```/g, '[Code example hidden based on your preferences]');
    }

    if (preferences.math === false) {
      // Simplify mathematical content
      filtered = filtered.replace(/\$\$[\s\S]*?\$\$|\$[^$]*\$/g, '[Mathematical expression simplified]');
      filtered = filtered.replace(/where \w+ =/g, 'where the value equals');
    }

    if (preferences.visual === false) {
      // Remove visual descriptions
      filtered = filtered.replace(/\(see diagram|as shown in|visual representation/gi, '(see the');
    }
  }

  return filtered;
};

const addPersonalizedExamples = (content, interests, chapterContext) => {
  if (!Array.isArray(interests) || interests.length === 0) return content;

  let personalized = content;

  // Add examples based on user interests
  if (interests.includes('humanoid') || interests.includes('humanoid robotics')) {
    personalized += `\n\n> **${chapterContext.title || 'Note'}**: For humanoid robots, this concept is particularly relevant when considering bipedal locomotion, balance control, and natural human-robot interaction.`;
  }

  if (interests.includes('navigation') || interests.includes('path planning')) {
    personalized += `\n\n> **Application**: This is crucial for autonomous navigation systems, especially for robots that need to operate in human environments.`;
  }

  if (interests.includes('manipulation') || interests.includes('grasping')) {
    personalized += `\n\n> **Use Case**: This technology enables robots to perform complex manipulation tasks such as object grasping, assembly, and tool use.`;
  }

  if (interests.includes('ai') || interests.includes('machine learning')) {
    personalized += `\n\n> **AI Connection**: This concept connects directly to machine learning applications in robotics, enabling adaptive and intelligent behavior.`;
  }

  return personalized;
};

// Component that can transform content based on user preferences
export const PersonalizedContentBlock = ({ children, context = {} }) => {
  const location = useLocation();
  const [transformedContent, setTransformedContent] = useState('');

  // Get user data from localStorage or default values
  const userProfile = getUserProfile();
  const chapterId = getChapterId(location.pathname);
  const preferences = getChapterPreferences(chapterId);

  // Transform content when it changes or user data changes
  useEffect(() => {
    const contentStr = typeof children === 'string' ? children : String(children);
    const transformed = transformContent(contentStr, userProfile, preferences, {
      ...context,
      chapterId
    });
    setTransformedContent(transformed);
  }, [children, userProfile, preferences, context, chapterId]);

  // Return the transformed content
  return <span>{transformedContent}</span>;
};

// Helper functions to get user data
const getUserProfile = () => {
  const defaultProfile = {
    name: '',
    interests: [],
    skillLevel: 'intermediate',
    domain: 'general',
    learningGoals: []
  };

  try {
    const savedProfile = localStorage.getItem('userProfile');
    return savedProfile ? JSON.parse(savedProfile) : defaultProfile;
  } catch (e) {
    console.warn('Could not load user profile:', e);
    return defaultProfile;
  }
};

const getChapterPreferences = (chapterId) => {
  if (!chapterId) return {};

  try {
    const savedPrefs = localStorage.getItem(`chapterPrefs_${chapterId}`);
    return savedPrefs ? JSON.parse(savedPrefs) : {};
  } catch (e) {
    console.warn('Could not load chapter preferences:', e);
    return {};
  }
};

const getChapterId = (pathname) => {
  const pathParts = pathname.split('/');
  return pathParts[pathParts.length - 1] || pathParts[pathParts.length - 2];
};

// Function to handle content transformation for entire chapters
export const transformWholeChapter = (chapterContent, chapterId, title) => {
  const userProfile = getUserProfile();
  const preferences = getChapterPreferences(chapterId);

  return transformContent(chapterContent, userProfile, preferences, {
    chapterId,
    title
  });
};