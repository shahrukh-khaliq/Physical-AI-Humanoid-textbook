import React from 'react';
import styles from './UnityCodeBlock.module.css';

interface UnityCodeBlockProps {
  children: React.ReactNode;
  type?: 'visual' | 'rendering' | 'interaction' | 'ros' | 'general';
  title?: string;
  description?: string;
}

const UnityCodeBlock: React.FC<UnityCodeBlockProps> = ({
  children,
  type = 'general',
  title,
  description
}) => {
  const typeLabels = {
    visual: 'Visual Rendering',
    rendering: 'Unity Rendering',
    interaction: 'Human-Robot Interaction',
    ros: 'ROS Integration',
    general: 'Code Example'
  };

  const typeStyles = {
    visual: styles.visual,
    rendering: styles.rendering,
    interaction: styles.interaction,
    ros: styles.ros,
    general: styles.general
  };

  return (
    <div className={`${styles.unityCodeBlock} ${typeStyles[type]}`}>
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

export default UnityCodeBlock;