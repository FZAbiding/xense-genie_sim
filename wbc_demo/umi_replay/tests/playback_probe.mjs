import fs from 'node:fs/promises';
import assert from 'node:assert/strict';

const [url, output, durationText = '15', action = 'observe'] = process.argv.slice(2);
assert(url && output && process.env.PLAYWRIGHT_MODULE, 'Set PLAYWRIGHT_MODULE and pass URL OUTPUT [SECONDS] [execute]');
const {chromium} = await import(process.env.PLAYWRIGHT_MODULE);
const browser = await chromium.launch({headless: true, args: ['--no-sandbox']});
await fs.mkdir(output, {recursive: true});
try {
  const page = await browser.newPage({viewport: {width: 1440, height: 1400}});
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  await page.goto(url, {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => document.getElementById('source-video').readyState >= 2);
  if (action === 'inspect') {
    await page.waitForFunction(() => [...document.querySelectorAll('[data-view]')].every(image => image.naturalWidth > 0));
    await page.waitForFunction(() => !document.querySelector('[data-command="reset_views"]').disabled);
    const state = async () => (await page.request.get(url + '/api/status')).json();
    const before = await state();
    const box = await page.locator('[data-orbit="0"]').boundingBox();
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width / 2 + 100, box.y + box.height / 2 + 25, {steps: 20});
    await page.mouse.up();
    await page.waitForTimeout(500);
    const dragged = await state();
    assert(Math.abs(dragged.camera_orbits[0][0] - before.camera_orbits[0][0]) > 0.1);
    await page.locator('[data-orbit="1"]').hover();
    await page.mouse.wheel(0, -180);
    await page.waitForTimeout(500);
    const zoomed = await state();
    assert(Math.abs(zoomed.camera_orbits[1][2] - dragged.camera_orbits[1][2]) > 0.1);
    await page.locator('[data-command="reset_views"]').click();
    await page.waitForTimeout(2500);
    const reset = await state();
    assert(Math.abs(reset.camera_orbits[0][0] - Math.PI / 4) < 1e-6);
    assert(Math.abs(reset.camera_orbits[1][2] - 1.9) < 1e-6);
    await page.screenshot({path: output + '/web-controls.png', fullPage: true});
    await fs.writeFile(output + '/controls.json', JSON.stringify({before, dragged, zoomed, reset, errors}, null, 2) + '\n');
    process.exitCode = errors.length ? 1 : 0;
  } else {
  await page.evaluate(() => {
    window.playbackProbe = {events: [], presentations: []};
    const video = document.getElementById('source-video');
    for (const kind of ['seeking', 'seeked', 'waiting', 'stalled', 'playing', 'pause']) {
      video.addEventListener(kind, () => window.playbackProbe.events.push({kind, wall_ms: performance.now(), media_s: video.currentTime}));
    }
    function presented(now, metadata) {
      window.playbackProbe.presentations.push({wall_ms: now, media_s: metadata.mediaTime});
      video.requestVideoFrameCallback(presented);
    }
    video.requestVideoFrameCallback(presented);
  });
  if (action === 'execute') {
    await page.waitForFunction(() => !document.getElementById('start').disabled);
    await page.locator('#start').click();
  }
  await page.waitForFunction(() => document.getElementById('stage').textContent === 'replaying', {timeout: 180000});
  await page.evaluate(() => { window.playbackProbe.events = []; window.playbackProbe.presentations = []; });
  const samples = [];
  for (let sample = 0; sample < Number(durationText); sample++) {
    await page.waitForTimeout(1000);
    samples.push(await page.evaluate(async () => {
      const video = document.getElementById('source-video');
      const begin = performance.now();
      const state = await (await fetch('/api/status', {cache: 'no-store'})).json();
      const quality = video.getVideoPlaybackQuality();
      return {wall_ms: performance.now(), status_rtt_ms: performance.now() - begin,
        state, video_time_s: video.currentTime, playback_rate: video.playbackRate,
        quality: {total: quality.totalVideoFrames, dropped: quality.droppedVideoFrames}};
    }));
  }
  const result = await page.evaluate(() => window.playbackProbe);
  result.samples = samples;
  result.errors = errors;
  result.browser = browser.version();
  await page.screenshot({path: output + '/web-playback.png', fullPage: true});
  await fs.writeFile(output + '/browser-probe.json', JSON.stringify(result, null, 2) + '\n');
  console.log(JSON.stringify({browser: result.browser, presentations: result.presentations.length,
    events: result.events, errors}, null, 2));
  }
} finally {
  await browser.close();
}
