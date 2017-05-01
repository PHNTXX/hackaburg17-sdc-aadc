const {Detector, Models} = require('snowboy');
const GoogleSpeech = require('@google-cloud/speech');
const SerialPort = require('serialport');
const PlaySound = require('play-sound');
const Aplay = require('aplay');
const AWS = require('aws-sdk');
const rec = require('node-record-lpcm16');
const fs = require('fs');

// config
const MAX_LISTEN_TIME = 10000;

// AWS Polly
const polly = new AWS.Polly({
  region: 'us-east-1'
});

// serial interfaces
// const ser0 = new SerialPort('/dev/ttyACM0', {baudRate: 115200});
// const ser1 = new SerialPort('/dev/ttyACM1', {baudRate: 115200});

// return a random element from the list
function randomPhrase(list) {
  // returns a random integer between min (included) and max (excluded)
  function getRandomInt(min, max) {
    min = Math.ceil(min);
    max = Math.floor(max);
    return Math.floor(Math.random() * (max - min)) + min;
  }

  return list[getRandomInt(0, list.length)];
}

// reply accordingly
function getResponseFromText(input) {
  console.log('generating reply...');

  if (input.match(/hello/g)) {
    return randomPhrase([
      'hello, Germans!'
    ]);
  }

  return null;
}

// speak some text with AWS Polly
function sayTextAws(text) {
  console.log('saying text with AWS Polly:', text);

  const params = {
    Text: text,
    OutputFormat: 'mp3',
    VoiceId: 'Brian'
  };

  const synthCallback = (err, data) => {
    if (err) {
      throw err;
    }

    fs.writeFile('./tmp/output.mp3', data.AudioStream, (err) => {
      if (err) {
        throw err;
      }

      console.log('wrote output.mp3');
      console.log('playing output...');

      const player = PlaySound();
      player.play('./tmp/output.mp3', function(err) {
        if (err) {
          throw err;
        }

        console.log('playing complete');
        listenForHotword();
      });
    });
  };

  polly.synthesizeSpeech(params, synthCallback);
}

// get the text from IBM STT and reply
function processTranscription(transcription) {
  console.log('processing transcription...');

  if (!transcription) {
    console.log('transcription is empty, skipping');
    listenForHotword();
    return;
  }

  const responseText = getResponseFromText(transcription.toLowerCase());
  if (responseText) {
    sayTextAws(responseText);
  } else {
    console.log('no response');
    listenForHotword();
  }
}

// stream audio to google, get back text
function streamToGoogle() {
  console.log('streaming to Google Speech API...');
  let transcription = '';
  let timeoutId = null;

  const speech = GoogleSpeech({
    projectId: 'okclaire-140417',
    keyFilename: '/home/pi/hackaburg/OkClaire-2952171eec99.json'
  });

  const options = {
    config: {
      encoding: 'LINEAR16',
      languageCode: 'en-US',
      sampleRateHertz: 16000
    }
  };

  rec.start({threshold: 0, sampleRate: 16000})
    .pipe(speech.createRecognizeStream(options))
    .on('data', (chunk) => {
      console.log('google chunk of data:', chunk);
      if (chunk && chunk.results) {
        transcription += chunk.results;
      }
      if (chunk && chunk.endpointerType === 'END_OF_SPEECH') {
        clearTimeout(timeoutId);
        console.log('Google Speech API detected end of speech...');
        new Aplay().play('./sounds/dong.wav');
        rec.stop();
      }
    })
    .on('end', () => {
      console.log('Google Speech API streaming stopped');
      console.log('transcription:', transcription);

      if (transcription.indexOf('speak') === 0) {
        sayTextAws(transcription.substr('speak'.length).trim());
      } else {
        processTranscription(transcription);
      }
    });

  timeoutId = setTimeout(() => {
    console.log('stopping Google Speech API stream...');
    new Aplay().play('./sounds/dong.wav');
    rec.stop();
  }, MAX_LISTEN_TIME);
}

// send data to serial
function sendSerial(data, ser, callback, callbackDelay) {
  ser.write(data, (err) => {
    if (err) {
      throw err;
    }

    console.log('sent ' + code + ' to ' + ser);

    if (callback) {
      setTimeout(callback, callbackDelay);
    }
  });
}

// listen for the hotword
function listenForHotword() {
  console.log('listening for hotword...');

  // Snowboy models
  const models = new Models();
  models.add({
    file: './snowboy/tindercar.pmdl',
    sensitivity: '0.6',
    hotwords: 'tindercar'
  });

  // Snowboy detector
  const detector = new Detector({
    resource: './snowboy/common.res',
    models: models,
    audioGain: 1.0
  });

  detector.on('hotword', (index, hotword) => {
    console.log('detected hotword', index, hotword);
    rec.stop();
    new Aplay().play('./sounds/ding.wav');
    streamToGoogle();
  });

  rec.start({threshold: 0}).pipe(detector);
}

// start
listenForHotword();
