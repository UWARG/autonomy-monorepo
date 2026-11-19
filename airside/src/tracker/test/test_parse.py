from tracker.tracker_bridge import load_csv, parse_payload, to_iso

GOOD = (b'{"session_id":"s1","sequence":5,"fix":true,'
        b'"gnss_time":"2026-11-08T19:30:39.423Z","latitude":50.1,"longitude":-110.7}')


def test_parse_good():
    session, seq, t, lat, lon = parse_payload(GOOD)
    assert (session, seq, lat, lon) == ("s1", 5, 50.1, -110.7)
    assert to_iso(t) == "2026-11-08T19:30:39.423Z"


def test_parse_skips_no_fix():
    assert parse_payload(GOOD.replace(b'"fix":true', b'"fix":false')) is None


def test_parse_skips_test_sample():
    assert parse_payload(GOOD.replace(b'"sequence":5', b'"sequence":0')) is None


def test_load_csv_missing_file(tmp_path):
    assert load_csv(str(tmp_path / "nope.csv")) == {}


def test_load_csv_roundtrip_and_skips_bad_line(tmp_path):
    p = tmp_path / "t.csv"
    p.write_text("timestamp,latitude,longitude\n"
                 "2026-11-08T19:30:39.423Z,50.0970884,-110.7325077\n"
                 "garbage\n")
    fixes = load_csv(str(p))
    assert list(fixes) == ["2026-11-08T19:30:39.423Z"]
    assert fixes["2026-11-08T19:30:39.423Z"][1:] == (50.0970884, -110.7325077)