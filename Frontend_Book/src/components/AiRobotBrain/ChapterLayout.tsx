import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './ChapterLayout.module.css';

interface ChapterLayoutProps {
  children: ReactNode;
  title: string;
  description?: string;
  learningObjectives?: string[];
  chapterNumber?: number;
  chapterTitle?: string;
}

const ChapterLayout: React.FC<ChapterLayoutProps> = ({
  children,
  title,
  description,
  learningObjectives,
  chapterNumber,
  chapterTitle,
}) => {
  return (
    <div className={styles.chapterContainer}>
      <header className={styles.chapterHeader}>
        <h1 className={styles.chapterTitle}>{title}</h1>
        {description && <p className={styles.chapterDescription}>{description}</p>}
      </header>

      {learningObjectives && learningObjectives.length > 0 && (
        <section className={styles.learningObjectives}>
          <h2>Learning Objectives</h2>
          <ul>
            {learningObjectives.map((objective, index) => (
              <li key={index}>{objective}</li>
            ))}
          </ul>
        </section>
      )}

      <main className={styles.chapterContent}>
        {children}
      </main>

      <footer className={styles.chapterFooter}>
        <div className={styles.chapterInfo}>
          {chapterNumber && chapterTitle && (
            <p>Chapter {chapterNumber}: {chapterTitle}</p>
          )}
        </div>
      </footer>
    </div>
  );
};

export default ChapterLayout;