import 'package:flutter/material.dart';
import 'books_details_section.dart';
import 'custom_book_details_appbar.dart';
import 'similar_books_section.dart';

class BookDetailsViewBody extends StatelessWidget {
  const BookDetailsViewBody({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomScrollView(
      slivers: [
        SliverFillRemaining(
          child: SafeArea(
            child: Column(
              children: [
                CustomBookDetailsAppBar(),
                BookDetailsSection(),
                Expanded(
                  child: SizedBox(
                    height: 20,
                  ),
                ),
                SimilarBooksSection()
              ],
            ),
          ),
        )
      ],
    );
  }
}
