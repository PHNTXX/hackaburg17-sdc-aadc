const {Detector, Models} = require('snowboy');
const rec = require('node-record-lpcm16');
const request = require('request');

const models = new Models();
models.add({file: './snowboy/tindercar.pmdl', sensitivity: '0.4', hotwords: 'tindercar'});
models.add({file: './snowboy/car.pmdl', sensitivity: '0.4', hotwords: 'car'});
models.add({file: './snowboy/left.pmdl', sensitivity: '0.4', hotwords: 'left'});
models.add({file: './snowboy/right.pmdl', sensitivity: '0.4', hotwords: 'right'});
models.add({file: './snowboy/stop.pmdl', sensitivity: '0.4', hotwords: 'stop'});
models.add({file: './snowboy/forward.pmdl', sensitivity: '0.4', hotwords: 'forward'});
models.add({file: './snowboy/backward.pmdl', sensitivity: '0.4', hotwords: 'backward'});
models.add({file: './snowboy/more.pmdl', sensitivity: '0.4', hotwords: 'more'});

const detector = new Detector({
  resource: './snowboy/common.res',
  models: models,
  audioGain: 1.0
});

detector.on('silence', () => {
  console.log('detected silence');
});

detector.on('sound', () => {
  console.log('detected sound');
});

detector.on('error', (err) => {
  console.log('detected error', err);
});

detector.on('hotword', (index, hotword) => {
  console.log('detected hotword', index, hotword);

  if (hotword === 'left') {
    request('http://bobbycar.local:8700/remote?dir=left', (err) => { if (err) { throw err; } });
  }
  if (hotword === 'right') {
    request('http://bobbycar.local:8700/remote?dir=right', (err) => { if (err) { throw err; } });
  }
  if (hotword === 'forward') {
    request('http://bobbycar.local:8700/remote?spd=forward', (err) => { if (err) { throw err; } });
  }
  if (hotword === 'backward') {
    request('http://bobbycar.local:8700/remote?spd=backward', (err) => { if (err) { throw err; } });
  }
  if (hotword === 'stop') {
    request('http://bobbycar.local:8700/remote?cmd=stop', (err) => { if (err) { throw err; } });
  }
  if (hotword === 'more') {
    request('http://bobbycar.local:8700/remote?cmd=more', (err) => { if (err) { throw err; } });
  }
});

// start
console.log('listening for hotwords...');
rec.start({threshold: 0}).pipe(detector);
