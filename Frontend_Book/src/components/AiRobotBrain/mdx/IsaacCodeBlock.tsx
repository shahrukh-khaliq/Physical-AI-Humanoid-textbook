import React from 'react';
import styles from './IsaacCodeBlock.module.css';

interface IsaacCodeBlockProps {
  children: React.ReactNode;
  type?: 'sim' | 'ros' | 'nav' | 'general';
  title?: string;
  description?: string;
}

const IsaacCodeBlock: React.FC<IsaacCodeBlockProps> = ({
  children,
  type = 'general',
  title,
  description
}) => {
  const typeLabels = {
    sim: 'Isaac Sim',
    ros: 'Isaac ROS',
    nav: 'Nav2',
    general: 'Code Example'
  };

  const typeStyles = {
    sim: styles.isaacSim,
    ros: styles.isaacRos,
    nav: styles.nav2,
    general: styles.general
  };

  return (
    <div className={`${styles.isaacCodeBlock} ${typeStyles[type]}`}>
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

export default IsaacCodeBlock;