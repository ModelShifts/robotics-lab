import json
from pathlib import Path

from robotics_lab.cli import main

ROOT = Path(__file__).resolve().parents[1]


def test_version(capsys):
    try:
        main(["--version"])
    except SystemExit as exc:
        assert exc.code == 0
    text = capsys.readouterr().out
    assert "robotics-lab" in text


def test_ros_analyze_and_reports(tmp_path, capsys):
    csv_path = tmp_path / "topics.csv"
    html_path = tmp_path / "report.html"
    code = main(["ros", "analyze", str(ROOT / "samples" / "ros" / "node_graph"), "--csv", str(csv_path), "--html", str(html_path)])
    assert code == 0
    out = capsys.readouterr().out
    assert "/joint_states" in out
    assert "5" in out
    assert csv_path.is_file()
    assert (tmp_path / "topics_edges.csv").is_file()
    html = html_path.read_text()
    assert "/joint_feedback" in html
    assert "ROS 2 bag report" in html


def test_ros_analyze_json(capsys):
    code = main(["ros", "analyze", str(ROOT / "samples" / "ros" / "node_graph"), "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["mcap_message_count"] == 600


def test_can_analyze(capsys):
    code = main(["can", "analyze", str(ROOT / "samples" / "can" / "trace.csv"), "--dbc", str(ROOT / "samples" / "can" / "robot.dbc")])
    assert code == 0
    assert "bus load" in capsys.readouterr().out


def test_papers_list(capsys):
    assert main(["papers", "list"]) == 0
    text = capsys.readouterr().out
    assert "Ijspeert" in text
    assert "Todorov" in text


def test_missing_bag_exits_2(capsys):
    assert main(["ros", "analyze", "samples/does-not-exist"]) == 2
    assert "error:" in capsys.readouterr().err


def test_papers_write_figures(tmp_path, capsys):
    assert main(["papers", "dmp", "--out", str(tmp_path / "dmp")]) == 0
    assert (tmp_path / "dmp" / "dmp_fit.png").is_file()
    assert main(["papers", "ilqr", "--out", str(tmp_path / "ilqr")]) == 0
    assert (tmp_path / "ilqr" / "ilqr_pendulum.png").is_file()
    assert (tmp_path / "ilqr" / "lqr_double_integrator.png").is_file()
    capsys.readouterr()
