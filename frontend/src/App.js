// App.js
import React, { useState, useEffect } from 'react';
import { Resizable } from 'react-resizable';
import HeaderMenu from './components/header_menu/HeaderMenu';
import FileBrowser from './components/file_browser/FileBrowser';
import FileEditor from './components/file_editor/FileEditor';
import './App.css';
import DiagramEditor from './components/diagram_editor/DiagramEditor';
import CommsManager from './libs/comms_manager';

const App = () => {

  const [editorWidth, setEditorWidth] = useState(700);
  const [currentFilename, setCurrentFilename] = useState('');
  const [currentProjectname, setCurrentProjectname] = useState('');
  const [modelJson, setModelJson] = useState('');
  const [projectChanges, setProjectChanges] = useState(false);

  const onResize = (key, size) => {
    switch (key) {
      case 'editorWidth':
        setEditorWidth(size.width);
        break;
      default:
        break;
    }
  };

  const ramHost = "127.0.0.1";
  const ramPort = 7163;
  const commsManagerInstance = CommsManager(`ws://${ramHost}:${ramPort}`);

  const bt_manager_host = "127.0.0.1";
  const bt_manager_port = 1905;
  const commsBTManager = CommsManager(`ws://${bt_manager_host}:${bt_manager_port}`);

  const [gazeboEnabled, setGazeboEnabled] = useState(false);

  useEffect(() => {

    const callback = (message) => {
      console.log(message.data.state);
      if (message.data.state === "ready") {
        setGazeboEnabled(true);
      }
    };

    commsManagerInstance.subscribe(
      [commsManagerInstance.events.STATE_CHANGED],
      callback
    );

    commsBTManager.subscribe(
      [commsBTManager.events.STATE_CHANGED],
      callback
    );
  }, []);

  return (
    <div className="App">

      <HeaderMenu 
        setCurrentProjectname={setCurrentProjectname} 
        currentProjectname={currentProjectname}
        modelJson={modelJson}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
        commsManagerInstance={commsManagerInstance}
        commsBTManager={commsBTManager}
      />

      <div className="App-main" style={{ display: 'flex' }}>

        <div style={{ width: '200px'}}>
          <FileBrowser 
            setCurrentFilename={setCurrentFilename} 
            currentFilename={currentFilename}
            currentProjectname={currentProjectname}
            setProjectChanges={setProjectChanges}
          />
        </div>
        
        <Resizable
          width={editorWidth}
          height={0}
          onResize={(e, { size }) => onResize('editorWidth', size)}
          minConstraints={[400, 400]}
          maxConstraints={[800, 800]}
        >
          <div style={{ width: `${editorWidth}px` }}>
            <FileEditor 
              currentFilename = {currentFilename} 
              currentProjectname={currentProjectname}
              setProjectChanges={setProjectChanges}
            />
          </div>
        </Resizable>

        <div style={{ flex: 1}}> 

          <div>

            <DiagramEditor 
              currentProjectname={currentProjectname}
              setModelJson={setModelJson}
              setProjectChanges={setProjectChanges}
            />
            
          </div>

          {gazeboEnabled ? (
            <div className='iframe-container'>
              <iframe
                id={"iframe"}
                src={"http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"}
              />
            </div>
          ):
          (
            <div className='iframe-container'>
              <h3>Loading simulation</h3>
            </div>
          )
          }

        </div>
        
      </div>

    </div>
  );
};

export default App;
