import React from 'react';
import styles from './IsaacDiagram.module.css';

interface IsaacDiagramProps {
  src: string;
  alt: string;
  caption?: string;
  type?: 'architecture' | 'workflow' | 'simulation' | 'pipeline' | 'general';
}

const IsaacDiagram: React.FC<IsaacDiagramProps> = ({
  src,
  alt,
  caption,
  type = 'general'
}) => {
  const typeLabels = {
    architecture: 'Architecture Diagram',
    workflow: 'Workflow',
    simulation: 'Simulation',
    pipeline: 'Processing Pipeline',
    general: 'Diagram'
  };

  return (
    <div className={`${styles.isaacDiagram} ${styles[type]}`}>
      <div className={styles.diagramContainer}>
        <img src={src} alt={alt} className={styles.diagramImage} />
      </div>
      <div className={styles.diagramInfo}>
        <div className={styles.caption}>
          <strong>{typeLabels[type]}:</strong> {caption || alt}
        </div>
      </div>
    </div>
  );
};

export default IsaacDiagram;