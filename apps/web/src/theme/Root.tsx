import React from 'react';
import Chat from '@site/src/components/Chat';

// Default implementation, that you can customize
export default function Root({children}: {children: React.ReactNode}): React.JSX.Element {
  return (
    <>
      {children}
      <Chat />
    </>
  );
}
