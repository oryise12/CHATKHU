"""Thin module runner for the prototype UI."""
def main(argv=None):
    import sys
    argv = sys.argv[1:] if argv is None else argv
    if '--webcam' in argv:
        from .webcam import main as run
    else:
        from .viewer import main as run
    return run(argv)

if __name__ == "__main__":
    raise SystemExit(main())
