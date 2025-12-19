import React from 'react';
import styles from './styles.module.css';

export default function GlowBlobs(): JSX.Element {
  return (
    <div className={styles.glowContainer}>
      <div className={styles.blob1}></div>
      <div className={styles.blob2}></div>
      <div className={styles.blob3}></div>
    </div>
  );
}
