import init, { WasmTamagotchi } from '../pkg/tamagotchi_wasm.js';

const width = 32, height = 16;
const framebuffer = new Array(width * height).fill(0);
const icons = {};
const steps_per_frame = 100;
const max_speed = 10;
let speed = 1;


function redraw() {
    const canvas = document.getElementById('screen');
    const ctx = canvas.getContext('2d');
    const imageData = ctx.createImageData(width, height);
    for (let i = 0; i < width * height; ++i) {
        const on = framebuffer[i];
        const idx = i * 4;
        imageData.data[idx + 0] = on ? 0 : 255;
        imageData.data[idx + 1] = on ? 0 : 255;
        imageData.data[idx + 2] = on ? 0 : 255;
        imageData.data[idx + 3] = 255;
    }
    ctx.putImageData(imageData, 0, 0);
}

window.js_screen_update = function() {
};

window.js_screen_set_pixel = function(x, y, value) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        framebuffer[y * width + x] = value ? 1 : 0;
    }
};

window.js_screen_set_icon = function(icon, value) {
    icons[icon] = value;
    // TODO: display icons
};

window.js_buzzer_set_frequency = function(freq) {
    window._buzzer_freq = freq;
};

window.js_buzzer_play = function(value) {
    if (!value) return;
    const freq = window._buzzer_freq || 1000;
    const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = 'sine';
    osc.frequency.value = freq;
    gain.gain.value = 1.2;
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    osc.start();
    setTimeout(() => {
        osc.stop();
        audioCtx.close();
    }, 100);
};


let timer = Date.now();
let tama = null;
const speedSlider = document.getElementById('speed-slider');
const speedValue = document.getElementById('speed-value');

function setupButtonEvents(tama) {
    const btnLeft = document.getElementById('btn-left');
    const btnMiddle = document.getElementById('btn-middle');
    const btnRight = document.getElementById('btn-right');

    if (btnLeft) {
        btnLeft.onmousedown = () => tama.set_button(0, true);
        btnLeft.onmouseup = () => tama.set_button(0, false);
        btnLeft.onmouseleave = () => tama.set_button(0, false);
    }
    if (btnMiddle) {
        btnMiddle.onmousedown = () => tama.set_button(1, true);
        btnMiddle.onmouseup = () => tama.set_button(1, false);
        btnMiddle.onmouseleave = () => tama.set_button(1, false);
    }
    if (btnRight) {
        btnRight.onmousedown = () => tama.set_button(2, true);
        btnRight.onmouseup = () => tama.set_button(2, false);
        btnRight.onmouseleave = () => tama.set_button(2, false);
    }


    window.addEventListener('keydown', (e) => {
        if (!tama) return;
        if (e.code === 'ArrowLeft') tama.set_button(0, true);
        if (e.code === 'ArrowUp') tama.set_button(1, true);
        if (e.code === 'ArrowRight') tama.set_button(2, true);
    });
    window.addEventListener('keyup', (e) => {
        if (!tama) return;
        if (e.code === 'ArrowLeft') tama.set_button(0, false);
        if (e.code === 'ArrowUp') tama.set_button(1, false);
        if (e.code === 'ArrowRight') tama.set_button(2, false);
        if (e.code === 'KeyS') set_speed(speed + 1);
    });
}

function set_speed(value) {
    speed = Math.max(parseInt(value, 10) % (max_speed + 1), 1);
    speedSlider.value = speed;
    speedValue.textContent = speed;
    if (tama) {
        tama.set_speed(speed);
    }
}


function mainloop() {
    if (tama == null) return;

    for (let i = 0; i < steps_per_frame * speed; i++) {
        tama.run_step();
    }
    let elapsed = Date.now() - timer;
    if (elapsed > (1000 / 60)) {
        redraw();
        timer = Date.now();
    }
    window.requestAnimationFrame(mainloop);
}

async function main() {
    await init();

    let romBuffer = null;
    const romInput = document.getElementById('romfile');
    const startBtn = document.getElementById('start-btn');
    const errorMsg = document.getElementById('error-msg');

    romInput.addEventListener('change', e => {
        console.log("change");
        const file = e.target.files[0];
        if (file) {
            file.arrayBuffer().then(buf => {
                romBuffer = new Uint8Array(buf);
                startBtn.disabled = false;
                errorMsg.textContent = '';
            });
        } else {
            romBuffer = null;
            startBtn.disabled = true;
            errorMsg.textContent = '';
        }
    });

    if (speedSlider) {
        speedSlider.max = max_speed;
        speedSlider.addEventListener('input', () => {
            set_speed(speedSlider.value);
        });
    }

    startBtn.addEventListener('click', () => {
        if (romBuffer) {
            errorMsg.textContent = '';

            tama = new WasmTamagotchi(romBuffer);

            setupButtonEvents(tama);

            window.requestAnimationFrame(mainloop);
        } else {
            errorMsg.textContent = 'No ROM loaded! Please upload a ROM file.';
        }
    });
}

main(); 