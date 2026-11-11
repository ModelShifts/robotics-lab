"""Record the local web demo. The server must already be running on port 7860.

Uses Playwright's video capture at 1280x800. ffmpeg turns that into
assets/demo.mp4 and assets/demo.gif. Screenshots land in /tmp/demo-shots.
"""

from __future__ import annotations

import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
SHOTS = Path("/tmp/demo-shots")
VIDEO_DIR = Path("/tmp/demo-video")


def main() -> int:
    SHOTS.mkdir(parents=True, exist_ok=True)
    if VIDEO_DIR.exists():
        for child in VIDEO_DIR.iterdir():
            child.unlink()
    VIDEO_DIR.mkdir(parents=True, exist_ok=True)

    with sync_playwright() as p:
        browser = p.chromium.launch(
            executable_path="/usr/local/bin/google-chrome",
            headless=True,
            args=["--no-sandbox", "--disable-dev-shm-usage"],
        )
        context = browser.new_context(
            viewport={"width": 1280, "height": 800},
            record_video_dir=str(VIDEO_DIR),
            record_video_size={"width": 1280, "height": 800},
        )
        page = context.new_page()
        page.goto("http://127.0.0.1:7860/", wait_until="networkidle", timeout=60000)
        page.get_by_text("MODELSHIFTS").wait_for(timeout=30000)
        page.wait_for_timeout(3500)
        page.screenshot(path=str(SHOTS / "ros.png"))

        page.get_by_role("tab", name="CAN bus").click()
        page.get_by_text("Bus load").first.wait_for(timeout=10000)
        page.wait_for_timeout(3000)
        page.screenshot(path=str(SHOTS / "can.png"))

        page.get_by_role("button", name="Record 0.5 s on the virtual bus").click()
        page.get_by_text("Recorded just now").wait_for(timeout=20000)
        page.wait_for_timeout(2000)
        page.screenshot(path=str(SHOTS / "can-live.png"))

        page.get_by_role("tab", name="Papers").click()
        page.get_by_text("computed in this process").wait_for(timeout=10000)
        page.wait_for_timeout(2500)
        page.screenshot(path=str(SHOTS / "papers.png"))
        page.mouse.wheel(0, 700)
        page.wait_for_timeout(2500)
        page.screenshot(path=str(SHOTS / "papers-scrolled.png"))

        video = page.video
        context.close()
        browser.close()
        if video is not None:
            saved = Path(video.path())
            target = VIDEO_DIR / "demo.webm"
            saved.replace(target)
            print(target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
