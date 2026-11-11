from robotics_lab.webapp import build_app, can_view, papers_view, ros_view


def test_views_produce_plots_and_tables():
    topics, edges, image, note = ros_view()
    assert any(row[0] == "/joint_states" for row in topics)
    assert any(row[0] == "/joint_states" and row[1] == "/joint_feedback" for row in edges)
    assert image.size[0] > 200
    assert "mcap" in note

    messages, _edges, image, note = can_view()
    assert {row[0] for row in messages} == {"JointState", "JointCommand", "JointFeedback"}
    assert image.size[0] > 200
    assert "Bus load" in note

    rows, dmp, pendulum, lqr, note = papers_view()
    assert len(rows) >= 5
    assert dmp.size[1] > 200 and pendulum.size[0] > 200 and lqr.size[0] > 200
    assert "synthetic" in note


def test_demo_builds():
    demo = build_app()
    assert demo is not None
