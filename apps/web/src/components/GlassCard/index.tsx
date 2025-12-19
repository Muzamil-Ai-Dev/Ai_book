import React from 'react';
import clsx from 'clsx';
import { motion } from 'framer-motion';
import styles from './styles.module.css';

interface GlassCardProps {
  children: React.ReactNode;
  className?: string;
  animate?: boolean;
}

export default function GlassCard({ children, className, animate = true }: GlassCardProps): JSX.Element {
  const CardContent = (
    <div className={clsx('glass', 'hover-glow', styles.glassCard, className)}>
      {children}
    </div>
  );

  if (!animate) return CardContent;

  return (
    <motion.div
      whileHover={{ scale: 1.02 }}
      transition={{ type: 'spring', stiffness: 300 }}
    >
      {CardContent}
    </motion.div>
  );
}
