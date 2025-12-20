import React from 'react';
import styles from './GazeboCodeBlock.module.css';

interface GazeboCodeBlockProps {
  children: React.ReactNode;
  type?: 'physics' | 'simulation' | 'sensor' | 'world' | 'general';
  title?: string;
  description?: string;
}

const GazeboCodeBlock: React.FC<GazeboCodeBlockProps> = ({
  children,
  type = 'general',
  title,
  description
}) => {
  const typeLabels = {
    physics: 'Physics Simulation',
    simulation: 'Gazebo Simulation',
    sensor: 'Sensor Configuration',
    world: 'World Building',
    general: 'Code Example'
  };

  const typeStyles = {
    physics: styles.physics,
    simulation: styles.simulation,
    sensor: styles.sensor,
    world: styles.world,
    general: styles.general
  };

  return (
    <div className={`${styles.gazeboCodeBlock} ${typeStyles[type]}`}>
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

export default GazeboCodeBlock;