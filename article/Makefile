all: paper.pdf

paper.pdf: paper.tex
	pdflatex paper.tex; bibtex paper.bib; pdflatex paper.tex; pdflatex paper.tex

clean:
	rm -f paper.dvi paper.ps *.snm *.out *.nav *.log *.aux *.toc *.vrb *.pdf *~ *.lof *.blg *.bbl
