import React from 'react';
import styles from './WhisperCodeBlock.module.css';

interface WhisperCodeBlockProps {
  children: React.ReactNode;
  type?: 'speech-recognition' | 'audio-processing' | 'transcription' | 'validation' | 'general';
  title?: string;
  description?: string;
}

const WhisperCodeBlock: React.FC<WhisperCodeBlockProps> = ({
  children,
  type = 'general',
  title,
  description
}) => {
  const typeLabels = {
    speech_recognition: 'Speech Recognition',
    audio_processing: 'Audio Processing',
    transcription: 'Transcription',
    validation: 'Command Validation',
    general: 'Code Example'
  };

  const typeStyles = {
    speech_recognition: styles.speechRecognition,
    audio_processing: styles.audioProcessing,
    transcription: styles.transcription,
    validation: styles.validation,
    general: styles.general
  };

  return (
    <div className={`${styles.whisperCodeBlock} ${typeStyles[type]}`}>
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

export default WhisperCodeBlock;