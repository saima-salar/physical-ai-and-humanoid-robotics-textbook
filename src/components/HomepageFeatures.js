import React from 'react';
import clsx from 'clsx';

import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Focus on Physical AI',
    Svg: require('@site/static/img/undraw_robotics.svg').default,
    description: (
      <>
        Dive deep into the principles and applications of physical artificial intelligence,
        exploring how robots perceive and interact with the real world.
      </>
    ),
  },
  {
    title: 'Comprehensive Humanoid Robotics',
    Svg: require('@site/static/img/undraw_robot_face.svg').default,
    description: (
      <>
        From kinematics to control systems, explore the intricacies of humanoid robot design,
        locomotion, and manipulation.
      </>
    ),
  },
  {
    title: 'Future-Proof Knowledge',
    Svg: require('@site/static/img/undraw_futuristic_interface.svg').default,
    description: (
      <>
        Stay ahead with insights into advanced topics, ethical considerations, and the
        exciting future of physical AI and humanoid robotics.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
