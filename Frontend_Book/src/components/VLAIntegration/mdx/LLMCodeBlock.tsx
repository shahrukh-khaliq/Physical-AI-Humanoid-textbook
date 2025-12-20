import React from 'react';
import styles from './LLMCodeBlock.module.css';

interface LLMCodeBlockProps {
  children: React.ReactNode;
  type?: 'planning' | 'prompt-engineering' | 'task-decomposition' | 'action-mapping' | 'general';
  title?: string;
  description?: string;
}

const LLMCodeBlock: React.FC<LLMCodeBlockProps> = ({
  children,
  type = 'general',
  title,
  description
}) => {
  const typeLabels = {
    planning: 'Cognitive Planning',
    prompt_engineering: 'Prompt Engineering',
    task_decomposition: 'Task Decomposition',
    action_mapping: 'Action Mapping',
    general: 'Code Example'
  };

  const typeStyles = {
    planning: styles.planning,
    prompt_engineering: styles.promptEngineering,
    task_decomposition: styles.taskDecomposition,
    action_mapping: styles.actionMapping,
    general: styles.general
  };

  return (
    <div className={`${styles.llmCodeBlock} ${typeStyles[type]}`}>
      <div className={styles.header}>
        <span className={styles.typeLabel}>{typeLabels[type]}</span>
        {title && <span className={styles.titleLabel}>{title}</span>}
      </div>
      {description && <p className={styles.description}>{description}</p>}
      <div className={styles.codeContent}>
        {children}
      </div>
    </div>
  );
};

export default LLMCodeBlock;