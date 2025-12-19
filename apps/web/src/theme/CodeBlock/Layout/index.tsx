import React, {type ReactNode} from 'react';
import clsx from 'clsx';
import {useCodeBlockContext} from '@docusaurus/theme-common/internal';
import Container from '@theme/CodeBlock/Container';
import Title from '@theme/CodeBlock/Title';
import Content from '@theme/CodeBlock/Content';
import type {Props} from '@theme/CodeBlock/Layout';
import Buttons from '@theme/CodeBlock/Buttons';

import styles from './styles.module.css';

export default function CodeBlockLayout({className}: Props): ReactNode {
  const {metadata} = useCodeBlockContext();
  return (
    <Container as="div" className={clsx(className, metadata.className, styles.codeBlockCustom)}>
      <div className={styles.codeBlockHeader}>
        <div className={styles.macOsButtons}>
          <span className={styles.macOsRed}></span>
          <span className={styles.macOsYellow}></span>
          <span className={styles.macOsGreen}></span>
        </div>
        <div className={styles.codeBlockTitleContainer}>
          {metadata.title ? (
            <Title>{metadata.title}</Title>
          ) : (
            <span className={styles.languageLabel}>{metadata.language}</span>
          )}
        </div>
        <div className={styles.codeBlockActionButtons}>
          <Buttons />
        </div>
      </div>
      <div className={styles.codeBlockContent}>
        <Content />
      </div>
    </Container>
  );
}
