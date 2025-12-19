import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { motion } from 'framer-motion';

import styles from './index.module.css';
import GlowBlobs from '../components/GlowBlobs';
import GlassCard from '../components/GlassCard';
import { fadeIn, staggerContainer } from '../utils/motion';

function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.hero}>
      <GlowBlobs />
      <motion.div 
        variants={staggerContainer(0.1, 0)}
        initial="hidden"
        whileInView="show"
        viewport={{ once: true, amount: 0.25 }}
        className="container"
      >
        <motion.div variants={fadeIn('up', 'spring', 0.1, 1)}>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
        </motion.div>
        
        <motion.p variants={fadeIn('up', 'spring', 0.2, 1)} className={styles.heroSubtitle}>
          Master the future of robotics with a spec-driven, AI-native approach to Physical AI and Humanoid systems.
        </motion.p>
        
        <motion.div variants={fadeIn('up', 'spring', 0.3, 1)} className={styles.ctaButtons}>
          <Link
            className="button button--primary button--lg"
            to="/modules/intro">
            Start Learning 🚀
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="https://github.com/Muzamil-Ai-Dev/Ai_book">
            View on GitHub
          </Link>
        </motion.div>
      </motion.div>
    </header>
  );
}

function FeaturesSection() {
  const features = [
    {
      title: 'Physical AI First',
      description: 'Go beyond traditional robotics. Learn how Large Language Models and Vision-Language-Action (VLA) models are transforming embodiment.',
      icon: '🤖',
    },
    {
      title: 'Simulation to Reality',
      description: 'Master NVIDIA Isaac, Gazebo, and Unity. Bridge the gap between digital twins and physical humanoid deployment.',
      icon: '🌐',
    },
    {
      title: 'Spec-Driven Learning',
      description: 'Every chapter is structured for technical precision. Clear objectives, reproducible exercises, and industry-grade standards.',
      icon: '📐',
    },
  ];

  return (
    <section className={styles.section}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Why this textbook?</Heading>
        <div className={styles.featuresGrid}>
          {features.map((props, idx) => (
            <GlassCard key={idx} className={styles.featureCard}>
              <span className={styles.featureIcon}>{props.icon}</span>
              <h3 className={styles.featureTitle}>{props.title}</h3>
              <p className={styles.featureDescription}>{props.description}</p>
            </GlassCard>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulesPreview() {
  const modules = [
    { id: '01', title: 'Intro to Physical AI', desc: 'The foundations of embodied intelligence.', slug: 'intro' },
    { id: '02', title: 'ROS 2 Fundamentals', desc: 'The industry-standard robotics middleware.', slug: 'ros2' },
    { id: '03', title: 'Simulation Environments', desc: 'Gazebo, Unity, and the importance of digital twins.', slug: 'gazebo-unity' },
    { id: '04', title: 'NVIDIA Isaac Platform', desc: 'Advanced AI-powered simulation and orchestration.', slug: 'isaac' },
    { id: '05', title: 'VLA Models', desc: 'Vision-Language-Action transformers for robotics.', slug: 'vla' },
    { id: '06', title: 'Humanoid Robotics', desc: 'Locomotion, control, and bipedal mechanics.', slug: 'humanoid' },
    { id: '07', title: 'Conversational Robotics', desc: 'Integrating Large Language Models for human-robot interaction.', slug: 'conversational' },
    { id: '08', title: 'Capstone Project', desc: 'Applying everything to a vertical slice humanoid deployment.', slug: 'capstone' },
  ];

  return (
    <section className={clsx(styles.section, styles.sectionAlt)}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Course Modules</Heading>
        <div className={styles.modulesList}>
          {modules.map((m, idx) => (
            <div key={idx} className={styles.moduleItem}>
              <div className={styles.moduleInfo}>
                <h3>Module {m.id}: {m.title}</h3>
                <p>{m.desc}</p>
              </div>
              <Link className="button button--outline button--primary" to={`/modules/${m.slug}`}>
                View
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="The definitive guide to Physical AI and Humanoid Robotics.">
      <main>
        <HeroSection />
        <FeaturesSection />
        <ModulesPreview />
      </main>
    </Layout>
  );
}
