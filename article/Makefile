all: book.pdf

book.pdf: book.tex
	pdflatex book.tex; bibtex book; pdflatex book.tex; pdflatex book.tex

clean:
	rm -f book.dvi book.ps *.snm *.out *.nav *.log *.aux *.toc *.vrb *.pdf *~ *.lof *.blg *.bbl
